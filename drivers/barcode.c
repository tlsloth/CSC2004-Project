#include "barcode.h"
#include <stdio.h>
#include <string.h> 
#include "pico/time.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// ==========================================================
// --- Internal Type Definitions ---
// ==========================================================
typedef enum { EDGE_NONE=0, EDGE_RISE, EDGE_FALL } edge_t;

// --- Pulse Data Structure (Renamed to fit library internal logic) ---
typedef struct {
    uint16_t dur_ms; 
    uint8_t is_bar; 
} seg_t;

// ==========================================================
// --- Global Variables (Merged Library State) ---
// ==========================================================

// ===== Edge detection state variables =====
static volatile uint8_t  raw_level, stable_level, prev_stable_level;
static volatile uint32_t last_change_ms;
static volatile edge_t   last_edge = EDGE_NONE; 

// ===== Segment structure variables =====
static seg_t win[9];
static int   win_len = 0;

// ===== Scanner state variables =====
typedef enum { SCAN_IDLE=0, SCAN_READ } scan_state_t;
static scan_state_t scan_state = SCAN_IDLE;
static char         decoded_msg[BARCODE_MAX_LEN];
static char         last_decoded_msg[BARCODE_MAX_LEN]; 
static uint32_t     last_activity_ms = 0;
static bool         scanning_active = false;
static barcode_callback_t user_callback = NULL;

// ===== Code-39 pattern table =====
typedef struct { char ch; const char *pat; } c39_pat_t;
static const c39_pat_t C39[] = {
    {'0',"nnnwwnwnn"},{'1',"wnnwnnnnw"},{'2',"nnwwnnnnw"},{'3',"wnwwnnnnn"},
    {'4',"nnnwwnnnw"},{'5',"wnnwwnnnn"},{'6',"nnwwwnnnn"},{'7',"nnnwnnwnw"},
    {'8',"wnnwnnwnn"},{'9',"nnwwnnwnn"},{'A',"wnnnnwnnw"},{'B',"nnwnnwnnw"},
    {'C',"wnwnnwnnn"},{'D',"nnnnwwnnw"},{'E',"wnnnwwnnn"},{'F',"nnwnwwnnn"},
    {'G',"nnnnnwwnw"},{'H',"wnnnnwwnn"},{'I',"nnwnnwwnn"},{'J',"nnnnwwwnn"},
    {'K',"wnnnnnnww"},{'L',"nnwnnnnww"},{'M',"wnwnnnnwn"},{'N',"nnnnwnnww"},
    {'O',"wnnnwnnwn"},{'P',"nnwnwnnwn"},{'Q',"nnnnnnwww"},{'R',"wnnnnnwwn"},
    {'S',"nnwnnnwwn"},{'T',"nnnnwnwwn"},{'U',"wwnnnnnnw"},{'V',"nwwnnnnnw"},
    {'W',"wwwnnnnnn"},{'X',"nwnnwnnnw"},{'Y',"wwnnwnnnn"},{'Z',"nwwnwnnnn"},
    {'-',"nwnnnnwnw"},{'.',"wwnnnnwnn"},{' ',"nwwnnnwnn"},{'*',"nwnnwnwnn"},
    {'$',"nwnwnwnnn"},{'/',"nwnwnnnwn"},{'+',"nwnnnwnwn"},{'%',"nnnwnwnwn"},
    {0, NULL}
};

// --- Forward Declarations for *Static* Helper Functions ---
static inline void reset_window(void);
static inline uint16_t build_mask_top3_from(const seg_t a[9]);
static bool lookup_mask(uint16_t mask9, char *out);
static inline void reset_scan_state(const char *why);
static void push_segment(uint16_t dur_ms, bool ended_is_bar);
static void barcode_on_decode_complete(const char *decoded, barcode_command_t cmd);
static bool timer_cb(struct repeating_timer *t);
static struct repeating_timer barcode_timer;
static barcode_command_t barcode_parse_command_local(const char* str);

// ==========================================================
// --- Code-39 Function Implementations ---
// ==========================================================
static inline void reset_window(void) { 
    win_len = 0; 
}

static inline uint16_t pat_to_mask(const char *p) {
    uint16_t m = 0;
    for (int i=0; i<9 && p[i]; ++i) 
        if (p[i]=='w'||p[i]=='W') 
            m |= (1u << (8 - i));
    return m;
}

static inline uint16_t build_mask_top3_from(const seg_t a[9]) {
    int idx[9]; 
    uint16_t d[9];
    
    for (int k=0; k<9; k++) { 
        idx[k] = k; 
        d[k] = a[k].dur_ms; 
    }
    
    // Partial selection sort - bring top 3 to front
    for (int pos=0; pos<3; ++pos) {
        int max_i = pos;
        for (int j=pos+1; j<9; ++j) 
            if (d[j] > d[max_i]) 
                max_i = j;
        
        if (max_i != pos) {
            uint16_t td = d[pos]; 
            d[pos] = d[max_i]; 
            d[max_i] = td;
            
            int ti = idx[pos];    
            idx[pos] = idx[max_i]; 
            idx[max_i] = ti;
        }
    }
    
    // Build 9-bit mask (MSB = first element; 1 = wide)
    uint16_t mask9 = 0;
    mask9 |= (1u << (8 - idx[0]));
    mask9 |= (1u << (8 - idx[1]));
    mask9 |= (1u << (8 - idx[2]));
    
    return mask9;
}

static bool lookup_mask(uint16_t mask9, char *out) {
    for (int i=0; C39[i].ch; ++i) {
        if (pat_to_mask(C39[i].pat) == mask9) { 
            *out = C39[i].ch; 
            return true; 
        }
    }
    return false;
}

static inline void reset_scan_state(const char *why) {
    // If we were actively reading a scan, notify application that scan ended (abort/timeout)
    if (scan_state == SCAN_READ) {
        // application hook - safe to call (main.c implements as flag setter)
        barcode_scan_finished();
    }

    reset_window();
    scan_state = SCAN_IDLE;
    decoded_msg[0] = '\0';
    if (why) {
        printf("[BARCODE] Reset: %s\n", why);
        fflush(stdout);
    }
}

// ===== Segment processing =====
static void push_segment(uint16_t dur_ms, bool ended_is_bar) {
    if (dur_ms == 0) return;

    // Start only on a BAR (sequence must start with a bar)
    if (win_len == 0) {
        if (!ended_is_bar) {
            // ignore leading spaces (do not log as "accepted")
            return;
        }
    } else {
        // Enforce alternation: B,S,B,S,...,B
        bool expect_bar = ((win_len % 2) == 0);
        if (expect_bar != ended_is_bar) {
            reset_scan_state("mismatched bar/space sequence");
            return;
        }
    }

    // Append segment
    win[win_len] = (seg_t){dur_ms, (uint8_t)ended_is_bar};

    // Debug log AFTER validation/appending
    //printf("[DEBUG_SEGMENT] Segment %d Accepted: %s, Dur=%u ms\n",
           //win_len, ended_is_bar ? "BAR" : "SPACE", dur_ms);
    fflush(stdout);

    win_len++;

    // If this is the first accepted segment, mark activity and notify application
    if (scan_state == SCAN_IDLE) {
        scan_state = SCAN_READ;
        last_activity_ms = to_ms_since_boot(get_absolute_time());
        barcode_scan_started(); // now called when we actually start scanning anything
    }

    if (win_len < 9) return;

    // 9 segments collected → classify
    char ch = 0;
    uint16_t mask = build_mask_top3_from(win);
    bool ok = lookup_mask(mask, &ch);
    reset_window();

    if (!ok) {
        reset_scan_state("unknown 9-bit pattern");
        return;
    }

    // State machine
    if (scan_state == SCAN_IDLE) {
        // If somehow we are still idle, only accept '*' to start (defensive)
        if (ch == '*') {
            decoded_msg[0] = '\0';
            scan_state = SCAN_READ;
            printf("[BARCODE] START detected\n");
            fflush(stdout);
        }
        return;
    }

    // SCAN_READ state
    if (ch == '*') {
        if(strlen(decoded_msg) == 0) {
            // Ignore stray stop if no data
            printf("[BARCODE] START detected, waiting for message.\n");
            return;
        }
        else{
            printf("[BARCODE] STOP detected\n");
            printf("[BARCODE] Decoded: \"%s\"\n", decoded_msg);
            fflush(stdout);
            barcode_on_decode_complete(decoded_msg, barcode_parse_command_local(decoded_msg));
        }
    }

    // Append character
    size_t L = strlen(decoded_msg);
    if (L + 2 < sizeof(decoded_msg)) { 
        decoded_msg[L] = ch; 
        decoded_msg[L+1] = '\0'; 
    }
}

// ===== Timer callback (1kHz sampling) =====
static bool timer_cb(struct repeating_timer *t) {
    if (!scanning_active) return true;
    
    raw_level = gpio_get(BARCODE_IR_DO_PIN);
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    static uint8_t  pending_level = 0xFF;
    static uint32_t pending_since = 0;

    // 1. Check for stable level change
    if (raw_level == stable_level) { 
        pending_level = 0xFF; 
        return true; 
    }
    
    if (pending_level != raw_level) { 
        pending_level = raw_level; 
        pending_since = now_ms; 
        return true; 
    }
    
    // 2. Apply Debounce/Verify Period
    if ((now_ms - pending_since) < BARCODE_VERIFY_MS) 
        return true;

    // --- DEBUG LOG: Edge Detected/Verified ---
    // printf("[DEBUG_EDGE] Edge Verified: %s -> %s, Stable for %u ms\n",
    //        stable_level ? "HIGH" : "LOW", raw_level ? "HIGH" : "LOW", BARCODE_VERIFY_MS);
    fflush(stdout);
    // --- END DEBUG LOG ---

    // 3. Accept change
    prev_stable_level = stable_level;
    stable_level = pending_level;

    uint32_t width_ms = (pending_since > last_change_ms) ? 
                        (pending_since - last_change_ms) : 0;

    // Map ended level to bar/space
    // Ended state is the state *before* the change (prev_stable_level)
    bool ended_was_low = (stable_level > prev_stable_level); 
    bool ended_is_bar  = (BARCODE_BAR_IS_LOW) ? ended_was_low : !ended_was_low;

    if (width_ms > 0 && width_ms < 65535) {
        push_segment((uint16_t)width_ms, ended_is_bar);
        last_activity_ms = pending_since;
    }

    last_change_ms = pending_since;
    return true;
}

// ===== Public API (Implementations) =====
void barcode_init_local(void) {
    gpio_init(BARCODE_IR_DO_PIN);
    gpio_set_dir(BARCODE_IR_DO_PIN, GPIO_IN);
    gpio_pull_up(BARCODE_IR_DO_PIN);

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    raw_level = stable_level = prev_stable_level = gpio_get(BARCODE_IR_DO_PIN);
    last_change_ms = now_ms;
    last_activity_ms = now_ms;
    
    decoded_msg[0] = '\0';
    last_decoded_msg[0] = '\0';
    
    printf("[BARCODE] Initialized on GPIO %d\n", BARCODE_IR_DO_PIN);
    fflush(stdout);
}

void barcode_set_scanning_active_local(bool active) {
    if (active == scanning_active) return; // No change

    scanning_active = active;

    if (scanning_active) {
        // Start scanning
        reset_window();
        scan_state = SCAN_IDLE;
        decoded_msg[0] = '\0';

        // record activity time so timeout logic has a baseline
        last_activity_ms = to_ms_since_boot(get_absolute_time());

        add_repeating_timer_ms(-BARCODE_SAMPLE_MS, timer_cb, NULL, &barcode_timer); 
        
        printf("[BARCODE] Scanning enabled (Polling every %dms)\n", BARCODE_SAMPLE_MS);
        fflush(stdout);
    } else {
        // Stop scanning
        cancel_repeating_timer(&barcode_timer);
        reset_scan_state("scanning disabled");
        printf("[BARCODE] Scanning disabled.\n");
        fflush(stdout);
    }
}

void barcode_set_callback_local(barcode_callback_t callback) {
    user_callback = callback;
}

static barcode_command_t barcode_parse_command_local(const char* str) {
    if (!str || !str[0]) return CMD_NONE;
    if (str[0] >= 'A' && str[0] <= 'Z') {
        if ((str[0] - 'A' + 1) % 2 != 0) {
            return CMD_RIGHT;
        } else {
            return CMD_LEFT;
        }
    }
    return CMD_NONE;
}

// Centralized decode-complete handler
static void barcode_on_decode_complete(const char *decoded, barcode_command_t cmd) {
    // existing decode handling: call user callback
    if (user_callback) {
        user_callback(decoded, cmd);
    }
    strncpy(last_decoded_msg, decoded, BARCODE_MAX_LEN);

    // Reset internal read state and notify application from reset_scan_state
    reset_scan_state("decode complete");
}

// Periodic update: handle timeouts/abort
void barcode_update_local(void) {
    if (!scanning_active) return;

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // If we've been reading and there's been no activity for BARCODE_RESET_MS, abort scan
    if (scan_state == SCAN_READ) {
        if ((now_ms - last_activity_ms) > BARCODE_RESET_MS) {
            // on timeout, ensure we stop and notify app via reset_scan_state
            reset_scan_state("scan timeout");
        }
    }
}


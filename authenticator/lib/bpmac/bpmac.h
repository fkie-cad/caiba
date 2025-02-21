#pragma once

#define MAC_LEN 4
#define INT_SIZE sizeof(int)

typedef struct pre_ctx_t{

    unsigned char mac_key[16];
    int default_msg[MAC_LEN/INT_SIZE];  // contains (bit_tags of message 0) XOR masking_tag
    int res[MAC_LEN/INT_SIZE];  // contains (bit_tags of message 0). Used to reset default_msg before adding masking_tag
    int* bit_flips; // points to (bit_tag_0^i XOR bit_tag_1^i) for all i bits of a potential message
    int max_len;
    int bit_index;

    uint8_t nonce_cache[16];
    uint8_t prev_nonce[16];
    uint8_t nonce_key[32];

} bpmac_ctx_t;

void bpmac_init(char* key, char* nonce_key, int max_size, bpmac_ctx_t* ctx);
void bpmac_start(bpmac_ctx_t* ctx, char* tag);
void bpmac_update(bpmac_ctx_t* ctx, uint8_t input_bit, char* tag);
void bpmac_finish(bpmac_ctx_t* ctx, char* tag);
void bpmac_reset(bpmac_ctx_t* ctx, char* tag);
void bpmac_sign( bpmac_ctx_t* ctx, char* msg, int size, char* output) __attribute__ ((optimize(3)));
void bpmac_pre(bpmac_ctx_t* ctx, uint8_t nonce[16], char* tag);
int bpmac_vrfy(char* msg, int size, char* sig, int mac_size, bpmac_ctx_t* ctx);
void bpmac_deinit(bpmac_ctx_t* ctx);

void xor_tags(void* tag, void* value) __attribute__ ((optimize(3)));

void bpmac_test();

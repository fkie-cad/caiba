#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <limits.h>

#include <mbedtls/md.h>

#include <string.h>
#include <stdio.h>

#include <mbedtls/aes.h>

#include "bpmac.h"

#define MAC_LEN_IN_INT (MAC_LEN/sizeof(int))

static void pbuf(void *buf, int n, char *s)
{
    int i;
    char *cp = (char *)buf;

    if (n <= 0 || n >= 30)
        n = 30;

    if (s)
        printf("%s: ", s);

    for (i = 0; i < n; i++)
        printf("%02X", (unsigned char)cp[i]);
    printf("\n");
}

void bpmac_init( char* key,  char* nonce_key, int max_size, bpmac_ctx_t* ctx){

    uint32_t i,j;

    ctx->bit_index = 0;

    memset(ctx->res, 0, MAC_LEN);
    memset(ctx->default_msg, 0, MAC_LEN);

    memcpy( ctx->mac_key, key, 16 );
    memcpy( ctx->nonce_key, nonce_key, 16 );

    memset(ctx->prev_nonce, 0, 16);
    memset(ctx->nonce_cache, 0, 16);

    mbedtls_aes_context aes_ctx;
    mbedtls_aes_init(&aes_ctx);
    mbedtls_aes_setkey_enc(&aes_ctx, (const uint8_t *) ctx->mac_key, 128);


    ctx->max_len = max_size*8+1;

    uint8_t output0[32];
    uint8_t output1[32];
    uint8_t input[32] = {0}; /* this implementation does only work for small max_len values (<256) now. In our case, it is enough */

    ctx->bit_flips = (int*)malloc((max_size*8+1)*MAC_LEN);
    if(! ctx->bit_flips){
        printf("Error: Could not allocate memory for bitflips MACs\n");
    }

    for(i=0; i<max_size*8 +1; i++){

        input[0] = 2*i;

        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, input, output0);

        input[0] += 1;

        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, input, output1);

        for(j=0; j<MAC_LEN_IN_INT; j++){
            ctx->default_msg[j] ^= ((int*)output0)[j];

            ctx->bit_flips[  i*MAC_LEN_IN_INT + j] = ((int*)output0)[j] ^ ((int*)output1)[j];
        }
    }

    /* store XOR of all bit tags to reset default_msg for next masking tag */
    memcpy(ctx->res, ctx->default_msg, MAC_LEN);
    mbedtls_aes_free(&aes_ctx);

}


inline void xor_tags(void* tag, void* value) {

#if (MAC_LEN == 4)
    *((uint32_t *)tag) ^= *((uint32_t *)value);
#elif (MAC_LEN == 8)
    *((uint64_t *)tag) ^= *((uint64_t *)value);
#elif (MAC_LEN == 12)
    ((uint64_t *)tag)[0] ^= ((uint64_t *)value)[0];
    ((uint32_t *)tag)[2] ^= ((uint32_t *)value)[2];
#elif (MAC_LEN == 16)
    ((uint64_t *)tag)[0] ^= ((uint64_t *)value)[0];
    ((uint64_t *)tag)[1] ^= ((uint64_t *)value)[1];
#endif

}

/**
 * ( Not needed anymore, as these steps are done in bpmac_pre().
 * Prepares bit index and reserves/clears memory for MAC computation
 * @param ctx BPMAC context
 * @param tag pointer to future MAC value for memory preparation
 */
void bpmac_start(bpmac_ctx_t* ctx, char* tag) {
    ctx->bit_index = 0;
    memcpy(tag, ctx->res, MAC_LEN);
}

/**
 * Performs xor operation of the bit tag of given bit with the partial MAC
 * @param ctx BPMAC context
 * @param input_bit bit value of current bit, either 0 or 1
 * @param tag partial MAC value
 */
void bpmac_update(bpmac_ctx_t* ctx, uint8_t input_bit, char* tag) {
    if (input_bit) {
        xor_tags(tag, &ctx->bit_flips[ctx->bit_index]);
    }
    ctx->bit_index += MAC_LEN_IN_INT;
}

/**
 * Finalizes the BPMAC value with one padding bit
 * @param ctx BPMAC context
 * @param tag MAC value which shall be finished
 */
void bpmac_finish(bpmac_ctx_t* ctx, char* tag) {
    xor_tags(tag, &ctx->bit_flips[ctx->bit_index]);
}

void bpmac_reset(bpmac_ctx_t* ctx, char* tag) {
    ctx->bit_index = 0;
    memcpy(tag, ctx->default_msg, MAC_LEN);
}

/**
 * Performs bpmac_update() and bpmac_finish() on given message.
 * @param ctx BPMAC context
 * @param msg Message to be signed
 * @param len Length of message
 * @param tag MAC tag that shall contain the BPMAC value
 */
void bpmac_sign(bpmac_ctx_t* ctx, char* msg, int len,  char* tag) {

    register int i,j;

    /* For each byte in the message*/
    for(i=0; i < len; ++i){
        /* For each bit in that byte*/

        for(j=0; j < 8; ++j){

            /* If that bit is set */
            if( msg[i] & (1<<(7-j)) ){

                /* current MAC XOR bitflip MAC */
                xor_tags( tag, &ctx->bit_flips[ctx->bit_index] );

            }

            ctx->bit_index += MAC_LEN_IN_INT; // Optimization: Computing the index like this, and not more complicatly only when bit is set, is on average slightly faster and decreases variance


        }
    }

    /* Add 1 padding bit */
    xor_tags( tag, &(ctx->bit_flips[ctx->bit_index]) );

}

/**
 * Computes the masking tag based on the given nonce and initializes the MAC tag with XOR of bit tags and masking tag.
 * Has to be called one time for each BPMAC computation before bpmac_update() or bpmac_sign().
 * @param ctx BPMAC context
 * @param nonce nonce used for masking tag. Has to be incremented or changed after use.
 * @param tag MAC tag that shall contain the MAC value
 */
void bpmac_pre(bpmac_ctx_t* ctx, uint8_t nonce[16], char* tag)
{
    /* 'index' indicates that we'll be using the 0th or 1st eight bytes
     * of the AES output. If last time around we returned the index-1st
     * element, then we may have the result in the cache already.
     */

#if (MAC_LEN == 4)
#define LOW_BIT_MASK 3
#elif (MAC_LEN == 8)
    #define LOW_BIT_MASK 1
#elif (MAC_LEN > 8)
#define LOW_BIT_MASK 0
#endif

    uint8_t tmp_nonce_lo[8];

#if (MAC_LEN < 12)
    int index = nonce[0] & LOW_BIT_MASK;
#else
    int index = 0;
#endif
    *(uint64_t *)tmp_nonce_lo = ((uint64_t *)nonce)[0];
    tmp_nonce_lo[0] &= ~LOW_BIT_MASK; /* zero last bit */

    if ( (((uint64_t *)tmp_nonce_lo)[0] != ((uint64_t *)ctx->prev_nonce)[0]) ||
         (((uint64_t *)nonce)[1] != ((uint64_t *)ctx->prev_nonce)[1]) )
    {
        ((uint64_t *)ctx->prev_nonce)[1] = ((uint64_t *)nonce)[1];
        ((uint64_t *)ctx->prev_nonce)[0] = ((uint64_t *)tmp_nonce_lo)[0];

        mbedtls_aes_context aes_ctx;
        mbedtls_aes_init(&aes_ctx);
        mbedtls_aes_setkey_enc(&aes_ctx, (const uint8_t *) ctx->nonce_key, 128);
        mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, (const uint8_t *) nonce, ctx->nonce_cache );
        mbedtls_aes_free(&aes_ctx);
    }

    /* reset default_msg to XOR of bit tags before adding masking tag */
    memcpy(ctx->default_msg, ctx->res, MAC_LEN);

    int k;
    for(k=0; k<MAC_LEN_IN_INT; k++){
        ((int*)ctx->default_msg)[k] ^= ((int*)ctx->nonce_cache)[k+index*MAC_LEN_IN_INT];
    }
    ctx->bit_index = 0;
    memcpy(tag, ctx->default_msg, MAC_LEN);
}

int bpmac_vrfy( char* msg, int size, char* sig, int mac_size, bpmac_ctx_t* ctx){

    char output[32];

    bpmac_sign(ctx, msg, size, output);

    return memcmp( sig, output, 16 );
}

void bpmac_deinit(bpmac_ctx_t* ctx){

    free(ctx->bit_flips);

}

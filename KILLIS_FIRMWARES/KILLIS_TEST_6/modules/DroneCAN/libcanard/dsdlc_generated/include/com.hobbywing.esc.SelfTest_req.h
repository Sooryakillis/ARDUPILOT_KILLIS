#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define COM_HOBBYWING_ESC_SELFTEST_REQUEST_MAX_SIZE 0
#define COM_HOBBYWING_ESC_SELFTEST_REQUEST_SIGNATURE (0xC48D4DE61C5295DFULL)
#define COM_HOBBYWING_ESC_SELFTEST_REQUEST_ID 216

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class com_hobbywing_esc_SelfTest_cxx_iface;
#endif

struct com_hobbywing_esc_SelfTestRequest {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = com_hobbywing_esc_SelfTest_cxx_iface;
#endif
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t com_hobbywing_esc_SelfTestRequest_encode(struct com_hobbywing_esc_SelfTestRequest* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool com_hobbywing_esc_SelfTestRequest_decode(const CanardRxTransfer* transfer, struct com_hobbywing_esc_SelfTestRequest* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _com_hobbywing_esc_SelfTestRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_hobbywing_esc_SelfTestRequest* msg, bool tao);
static inline bool _com_hobbywing_esc_SelfTestRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_hobbywing_esc_SelfTestRequest* msg, bool tao);
void _com_hobbywing_esc_SelfTestRequest_encode(uint8_t* buffer, uint32_t* bit_ofs, struct com_hobbywing_esc_SelfTestRequest* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

}

/*
 decode com_hobbywing_esc_SelfTestRequest, return true on failure, false on success
*/
bool _com_hobbywing_esc_SelfTestRequest_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct com_hobbywing_esc_SelfTestRequest* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;
    return false; /* success */
}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct com_hobbywing_esc_SelfTestRequest sample_com_hobbywing_esc_SelfTestRequest_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
#endif
#endif

#ifndef MC_PLATFORM_COMPILER_H
#define MC_PLATFORM_COMPILER_H

#if defined(__IAR_SYSTEMS_ICC__)
#define MC_INLINE inline
#define MC_RESTRICT restrict
#define MC_ALIGN(bytes_) _Pragma("data_alignment=" #bytes_)
#define MC_UNUSED __root
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define MC_INLINE __inline
#define MC_RESTRICT __restrict
#define MC_ALIGN(bytes_) __align(bytes_)
#define MC_UNUSED __attribute__((unused))
#elif defined(__ghs__)
#define MC_INLINE inline
#define MC_RESTRICT restrict
#define MC_ALIGN(bytes_) __attribute__((aligned(bytes_)))
#define MC_UNUSED __attribute__((unused))
#else
#define MC_INLINE inline
#define MC_RESTRICT restrict
#define MC_ALIGN(bytes_) __attribute__((aligned(bytes_)))
#define MC_UNUSED __attribute__((unused))
#endif

#define MC_STATIC_ASSERT(cond_, name_) typedef char name_[(cond_) ? 1 : -1]

#endif /* MC_PLATFORM_COMPILER_H */

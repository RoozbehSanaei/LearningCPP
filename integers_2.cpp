/* Contents:
   1) Promotions (short/unsigned short) and storing back
   2) Usual arithmetic conversions (unforced, platform-dependent)
   3) Unsigned -> signed conversion hazards (out-of-range)
   4) Division/remainder sign rules + identity + INT_MIN%-1 hazard (gated)
   5) Unary minus on minimum value (gated UB)
   6) Right shift of negative signed values (implementation-defined)
   7) Bit masks: good (unsigned) vs risky (signed) patterns
   8) Gated compile errors / UB / platform-dependent blocks
*/

// Set to 1 to intentionally include code that does not compile.
#define SHOW_COMPILE_ERRORS 0
// Set to 1 to run undefined-behavior examples (dangerous).
#define SHOW_UNDEFINED_BEHAVIOR 0
// Set to 1 to show results that may differ across platforms/compilers.
#define SHOW_PLATFORM_DEPENDENT 1

// Gives std::cout for printing.
#include <iostream>
// Gives INT_MIN/INT_MAX/LONG_MAX/UINT_MAX/etc.
#include <climits>

// Returns the bit-width of unsigned int without using sizeof.
unsigned int uint_bits()
{
    // Starts with all bits set (well-defined for unsigned).
    unsigned int x = ~0u;
    // Counts how many right shifts until zero.
    unsigned int bits = 0u;
    // Repeats while any bit is still set.
    while (x != 0u) { ++bits; x >>= 1; }
    // Returns bit count (often 32).
    return bits;
}

// Demonstrates promotions for small integer types and the risk of storing back.
void demo_promotions()
{
    // Creates a short near its typical upper range.
    short a = 30000;
    // Creates another short.
    short b = 1000;

    // Adds shorts; computation typically happens after promotion to int.
    int sum_as_int = a + b;
    // Prints: typically "short+short as int => 31000" (exact value depends on inputs).
    std::cout << "short+short as int => " << sum_as_int << "\n";

    // Stores the promoted result back into short (can be non-portable if out of range).
    short stored = (short)(a + b);
    // Prints: "stored back into short => <value>" (may vary if the true sum does not fit).
    std::cout << "stored back into short => " << stored << "\n";

    // Creates two unsigned short values.
    unsigned short ua = 60000u;
    // Creates another unsigned short.
    unsigned short ub = 6000u;

    // Adds unsigned shorts; computation typically happens after promotion to int.
    int us_sum = ua + ub;
    // Prints: "unsigned short+unsigned short as int => <value>" (value shown as int).
    std::cout << "unsigned short+unsigned short as int => " << us_sum << "\n";

    // Stores the sum back into unsigned short; if it exceeds USHRT_MAX it wraps/truncates in the destination representation (non-portable expectation).
    unsigned short us_stored = (unsigned short)(ua + ub);
    // Prints: "stored back into unsigned short => <value>" (may differ if out of range).
    std::cout << "stored back into unsigned short => " << us_stored << "\n";
}

// Demonstrates usual arithmetic conversions without forcing casts.
void demo_usual_arithmetic_conversions()
{
    // Creates a negative signed int.
    int si = -1;
    // Creates an unsigned int.
    unsigned int ui = 1u;

    // Compares signed and unsigned; signed may convert to unsigned first.
    bool cmp1 = (si < ui);
    // Prints: often "(-1 < 1u) => 0" (surprising).
    std::cout << "(-1 < 1u) => " << cmp1 << "\n";

#if SHOW_PLATFORM_DEPENDENT
    // Creates a signed long (width differs across platforms).
    long sl = -1L;
    // Creates an unsigned int.
    unsigned int u2 = 1u;

    // Compares long and unsigned int; the chosen common type can differ across platforms.
    bool cmp2 = (sl < u2);
    // Prints: "(-1L < 1u) => 0 or 1" (platform-dependent).
    std::cout << "(-1L < 1u) => " << cmp2 << "\n";

    // Creates an unsigned long (width differs across platforms).
    unsigned long ul = ULONG_MAX;
    // Creates a signed long long.
    long long sll = -1LL;

    // Compares unsigned long and signed long long; common type choice can be platform-dependent.
    bool cmp3 = (sll < ul);
    // Prints: "(-1LL < ULONG_MAX) => 0 or 1" (platform-dependent).
    std::cout << "(-1LL < ULONG_MAX) => " << cmp3 << "\n";
#endif
}

// Demonstrates unsigned->signed conversion hazards when the unsigned value does not fit.
void demo_unsigned_to_signed_hazards()
{
    // Takes the maximum unsigned int.
    unsigned int umax = UINT_MAX;
    // Converts to signed int; if out of range, the result is implementation-defined (non-portable).
    int converted = (int)umax;
    // Prints: often "UINT_MAX as int => -1" but this is not guaranteed.
    std::cout << "UINT_MAX as int => " << converted << "\n";

    // Takes a mid-range unsigned int that typically fits.
    unsigned int small = 123u;
    // Converts to int; this should be value-preserving on all normal systems where int can represent 123.
    int ok = (int)small;
    // Prints: "123u as int => 123".
    std::cout << "123u as int => " << ok << "\n";
}

// Demonstrates division/remainder sign behavior and a key identity.
void demo_divmod_sign_and_identity()
{
    // Picks a negative dividend.
    int a = -7;
    // Picks a positive divisor.
    int b = 3;

    // Computes quotient (truncates toward zero).
    int q = a / b;
    // Computes remainder (sign follows dividend).
    int r = a % b;

    // Prints: typically "a=-7 b=3 q=-2 r=-1".
    std::cout << "a=-7 b=3 q=" << q << " r=" << r << "\n";

    // Picks a negative divisor.
    int c = -3;

    // Computes quotient for 7 / -3.
    int q2 = 7 / c;
    // Computes remainder for 7 % -3 (remainder sign follows dividend: 7).
    int r2 = 7 % c;

    // Prints: typically "7/(-3) q=-2 r=1".
    std::cout << "7/(-3) q=" << q2 << " r=" << r2 << "\n";

    // Checks the identity: a == (a/b)*b + a%b (for b != 0 and defined division).
    int recon = (a / b) * b + (a % b);
    // Prints: "reconstruct => -7" (expected).
    std::cout << "reconstruct => " << recon << "\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // Computes INT_MIN % -1; tied to INT_MIN / -1 hazard (can be undefined behavior).
    int ub_mod = INT_MIN % -1;
    // Prints: unreliable (may trap on some systems).
    std::cout << "UB INT_MIN%-1 => " << ub_mod << "\n";
#endif
}

// Demonstrates unary minus overflow on the minimum signed value (gated UB).
void demo_unary_minus_min()
{
#if SHOW_UNDEFINED_BEHAVIOR
    // Unary minus of INT_MIN can overflow because +INT_MAX is smaller in magnitude than -INT_MIN.
    int ub = -INT_MIN;
    // Prints: unreliable (do not trust).
    std::cout << "UB -INT_MIN => " << ub << "\n";
#endif
}

// Demonstrates right shift of negative signed values (implementation-defined).
void demo_right_shift_negative()
{
#if SHOW_PLATFORM_DEPENDENT
    // Uses a negative signed value.
    int neg = -1;
    // Right shift of negative signed can be implementation-defined (arithmetic vs logical shift).
    int shifted = (neg >> 1);
    // Prints: often "-1" with arithmetic shift, but can differ (implementation-defined).
    std::cout << "-1>>1 => " << shifted << "\n";
#endif
}

// Demonstrates bit mask practice: unsigned is the safer “bit container”.
void demo_bit_masks()
{
    // Starts with no flags.
    unsigned int flags = 0u;
    // Builds a mask for bit 2 using unsigned shifting (well-defined with valid count).
    unsigned int bit2 = 1u << 2;
    // Sets bit 2.
    flags = flags | bit2;
    // Prints: "flags => 4".
    std::cout << "flags => " << flags << "\n";

    // Tests bit 2.
    bool has = (flags & bit2) != 0u;
    // Prints: "has bit2 => 1".
    std::cout << "has bit2 => " << has << "\n";

    // Clears bit 2.
    flags = flags & ~bit2;
    // Prints: "flags after clear => 0".
    std::cout << "flags after clear => " << flags << "\n";

#if SHOW_PLATFORM_DEPENDENT
    // Creates an all-ones mask in signed int; commonly -1, but tying bit meaning to signed representation is risky.
    int signed_all_ones = ~0;
    // Prints: often "-1" but representation assumptions are not a safe habit.
    std::cout << "~0 (signed int) => " << signed_all_ones << "\n";
#endif

#if SHOW_UNDEFINED_BEHAVIOR
    // Left shift into/through the sign bit for signed int can be undefined behavior.
    int ub = 1 << ((int)uint_bits() - 1);
    // Prints: unreliable (do not trust).
    std::cout << "UB signed 1<<topbit => " << ub << "\n";
#endif
}

// Program entry point.
int main()
{
    // Prints: heading.
    std::cout << "=== integer types: complement file ===\n";

    // Runs promotions demo.
    demo_promotions();

    // Runs usual arithmetic conversions demo.
    demo_usual_arithmetic_conversions();

    // Runs unsigned->signed hazard demo.
    demo_unsigned_to_signed_hazards();

    // Runs division/remainder demo.
    demo_divmod_sign_and_identity();

    // Runs unary minus minimum demo (gated).
    demo_unary_minus_min();

    // Runs right shift negative demo (platform-dependent).
    demo_right_shift_negative();

    // Runs bit mask demo.
    demo_bit_masks();

#if SHOW_COMPILE_ERRORS
    // Intentional compile error (missing initializer).
    int will_not_compile = ;
    // Prints: unreachable if compilation fails.
    std::cout << will_not_compile << "\n";
#endif

    // Returns success.
    return 0;
}

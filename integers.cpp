// ===============================================================
// Contents
//  1) Quick orientation: what “integer types” are in C++
//  2) Ranges: why short/int/long/long long differ across platforms
//  3) Signed vs unsigned: intended uses and typical pitfalls
//  4) Overflow/underflow: what is safe, what is dangerous
//  5) Unsigned wraparound: defined behavior that often causes bugs
//  6) Mixing signed/unsigned and different ranks: surprising results
//  7) Promotions: smaller integer types change in expressions
//  8) Division/modulo: truncation and undefined cases
//  9) Shifts: safe mask usage vs undefined behavior
// 10) Gated “bad examples”: compile errors, undefined behavior, platform-dependent behavior
// ===============================================================

// This header provides std::cout for printing results.
#include <iostream>
// This header provides portable integer limits macros like INT_MAX, LONG_MAX, etc.
#include <climits>

// This macro enables code that intentionally fails to compile when set to 1.
#define SHOW_COMPILE_ERRORS 0
// This macro enables code that may execute undefined behavior when set to 1.
#define SHOW_UNDEFINED_BEHAVIOR 0
// This macro enables code whose result can vary by platform/compiler when set to 1.
#define SHOW_PLATFORM_DEPENDENT 0

// This function adds two int values safely by checking for overflow/underflow first.
bool checked_add_int(int a, int b, int& out)
{
    // This detects positive overflow: a + b would exceed INT_MAX.
    if (b > 0 && a > INT_MAX - b) { return false; }
    // This detects negative underflow: a + b would go below INT_MIN.
    if (b < 0 && a < INT_MIN - b) { return false; }
    // This performs the addition only after it is known to be safe.
    out = a + b;
    // This reports success to the caller.
    return true;
}

// This function multiplies two int values safely by using a wider intermediate and then range-checking.
bool checked_mul_int(int a, int b, int& out)
{
    // This widens a into long long so the multiplication is done in a wider type.
    long long wide = a;
    // This multiplies in long long to reduce the chance of intermediate overflow for int inputs.
    wide = wide * b;
    // This rejects results that would not fit in int.
    if (wide < INT_MIN || wide > INT_MAX) { return false; }
    // This assignment is safe because the bounds check already guaranteed it fits.
    out = wide;
    // This reports success to the caller.
    return true;
}

// This function converts long long to int safely by checking bounds first.
bool checked_ll_to_int(long long v, int& out)
{
    // This rejects values smaller than the minimum representable int.
    if (v < INT_MIN) { return false; }
    // This rejects values larger than the maximum representable int.
    if (v > INT_MAX) { return false; }
    // This assignment is safe because the bounds check already guaranteed it fits.
    out = v;
    // This reports success to the caller.
    return true;
}

// This helper prints a signed range and the matching unsigned range using only portable macros.
void print_range_pair(const char* signed_name, long long signed_min, long long signed_max,
                      const char* unsigned_name, unsigned long long unsigned_max)
{
    // Output varies by platform: prints the signed type range.
    std::cout << signed_name << " range: [" << signed_min << ", " << signed_max << "]\n";
    // Output varies by platform: prints the unsigned type range (minimum is always 0).
    std::cout << unsigned_name << " range: [0, " << unsigned_max << "]\n";
}

// This is the entry point; execution begins here.
int main()
{
    // Output: Integer types (built-in) demo
    std::cout << "Integer types (built-in) demo\n";

    // Output: (blank line) Basic facts and ranges:
    std::cout << "\nBasic facts and ranges:\n";

    // Output often: CHAR_BIT = 8 (but it can vary on unusual targets)
    std::cout << "CHAR_BIT = " << CHAR_BIT << "\n";

    // This prints the short / unsigned short ranges (platform-dependent).
    print_range_pair("short", SHRT_MIN, SHRT_MAX, "unsigned short", USHRT_MAX);
    // This prints the int / unsigned int ranges (platform-dependent).
    print_range_pair("int", INT_MIN, INT_MAX, "unsigned int", UINT_MAX);
    // This prints the long / unsigned long ranges (notably platform-dependent for width).
    print_range_pair("long", LONG_MIN, LONG_MAX, "unsigned long", ULONG_MAX);
    // This prints the long long / unsigned long long ranges (long long is at least 64-bit).
    print_range_pair("long long", LLONG_MIN, LLONG_MAX, "unsigned long long", ULLONG_MAX);

    // Output: (blank line) Signed vs unsigned intent:
    std::cout << "\nSigned vs unsigned intent:\n";

    // This uses signed int for a quantity that can naturally go negative (general arithmetic).
    int signed_quantity = 10;
    // This uses unsigned int for a bit-mask/flags value (bit-pattern semantics).
    unsigned int flags = 0u;

    // Output: signed_quantity=10 flags=0
    std::cout << "signed_quantity=" << signed_quantity << " flags=" << flags << "\n";

    // This creates a mask with bit 2 set (commonly used with flags).
    unsigned int bit2 = 1u << 2;
    // This sets bit 2 inside flags using bitwise OR.
    flags = flags | bit2;

    // Output: flags after setting bit2 => 4 (if bit2 is the 3rd bit)
    std::cout << "flags after setting bit2 => " << flags << "\n";

    // Output: (blank line) Checked arithmetic (recommended practice):
    std::cout << "\nChecked arithmetic (recommended practice):\n";

    // This stores a value near the maximum int to demonstrate overflow prevention.
    int near_max = INT_MAX;
    // This stores a small addend.
    int addend = 1;
    // This is where the checked result will be placed if safe.
    int out = 0;
    // This attempts the checked add; it will report failure instead of overflowing.
    bool ok_add = checked_add_int(near_max, addend, out);

    // Output: checked_add_int(INT_MAX, 1) ok=0 (blocked)
    std::cout << "checked_add_int(INT_MAX, 1) ok=" << ok_add << " (blocked)\n";

    // This stores two values whose product often does not fit in 32-bit int.
    int m1 = 50'000;
    // This stores the second factor.
    int m2 = 50'000;
    // This attempts a checked multiply with a wider intermediate.
    bool ok_mul = checked_mul_int(m1, m2, out);

    // Output: checked_mul_int(50000, 50000) ok=0 (often blocked on 32-bit int)
    std::cout << "checked_mul_int(50000, 50000) ok=" << ok_mul << " (often blocked on 32-bit int)\n";

    // Output: (blank line) Unsigned wraparound (defined but often unintended):
    std::cout << "\nUnsigned wraparound (defined but often unintended):\n";

    // This sets an unsigned int to zero.
    unsigned int u0 = 0u;
    // This subtracts 1 from 0; it wraps to the maximum value for that unsigned width (unexpected if you expected -1).
    unsigned int wrap_under = u0 - 1u;

    // Output often: 0u - 1u => 4294967295 (if unsigned int is 32-bit), otherwise the max for that width
    std::cout << "0u - 1u => " << wrap_under << " (wraparound)\n";

    // This sets an unsigned int to its maximum representable value.
    unsigned int umax = UINT_MAX;
    // This adds 1; it wraps to 0 (unexpected if you expected a larger number).
    unsigned int wrap_over = umax + 1u;

    // Output often: UINT_MAX + 1u => 0 (wraparound)
    std::cout << "UINT_MAX + 1u => " << wrap_over << " (wraparound)\n";

    // Output: (blank line) Common unsigned bug pattern (count goes “negative”):
    std::cout << "\nCommon unsigned bug pattern (count goes “negative”):\n";

    // This represents a count using unsigned (often a mistake when decrements can cross below zero).
    unsigned int remaining = 0u;
    // This decrements an unsigned zero; it becomes a very large number (unexpected for a “remaining items” count).
    remaining = remaining - 1u;

    // Output often: remaining after decrementing 0 => 4294967295 (if unsigned int is 32-bit)
    std::cout << "remaining after decrementing 0 => " << remaining << " (unexpected huge value)\n";

    // Output: (blank line) Mixing signed and unsigned (surprising comparisons and results):
    std::cout << "\nMixing signed and unsigned (surprising comparisons and results):\n";

    // This is a negative signed int.
    int sneg = -1;
    // This is a small unsigned int.
    unsigned int upos = 1u;
    // This comparison is often surprising because the signed value can be converted to unsigned before comparing.
    bool cmp1 = (sneg < upos);

    // Output is commonly: (-1 < 1u) => 0 (false), which surprises many beginners
    std::cout << "(-1 < 1u) => " << cmp1 << " (often surprising)\n";

    // This demonstrates mixing *ranks*: unsigned long and int can change the effective type used.
    unsigned long big_rank = 1ul;
    // This is a negative int that will often be converted when combined with unsigned long.
    int neg_rank = -2;
    // This stores the result in unsigned long, showing how negatives can turn into very large values.
    unsigned long mixed_rank_sum = neg_rank + big_rank;

    // Output: typically a very large number (because -2 becomes a large unsigned value before addition)
    std::cout << "(-2 + 1ul) as unsigned long => " << mixed_rank_sum << " (unexpected large value)\n";

    // Output: (blank line) Promotions (smaller integer types change in expressions):
    std::cout << "\nPromotions (smaller integer types change in expressions):\n";

    // This creates an unsigned short near its typical maximum.
    unsigned short us1 = 65000u;
    // This creates another unsigned short.
    unsigned short us2 = 1000u;
    // This adds them; operands are typically promoted to int or unsigned int before addition.
    unsigned int promoted_us_sum = us1 + us2;

    // Output: promoted sum of unsigned short values => 66000 (typical), but promotion details depend on type ranges
    std::cout << "promoted sum of unsigned short values => " << promoted_us_sum << " (promotion happens)\n";

    // This creates a signed short.
    short sh1 = 30'000;
    // This creates another signed short.
    short sh2 = 10'000;
    // This addition is typically performed in int after promotion (so the computation can exceed short safely).
    int promoted_sh_sum = sh1 + sh2;

    // Output: short+short computed (typically) as int => 40000
    std::cout << "short+short computed (typically) as int => " << promoted_sh_sum << "\n";

    // Output: (blank line) Narrowing conversions (practice: check bounds before converting):
    std::cout << "\nNarrowing conversions (practice: check bounds before converting):\n";

    // This is a value that is larger than int can hold on typical platforms.
    long long big_value = 9'000'000'000LL;
    // This is where the converted int would go if it fits.
    int narrowed = 0;
    // This attempts a checked conversion to avoid an out-of-range int result.
    bool ok_narrow = checked_ll_to_int(big_value, narrowed);

    // Output: 9,000,000,000LL -> int ok=0 (blocked) on typical platforms with 32-bit int
    std::cout << "9,000,000,000LL -> int ok=" << ok_narrow << " (blocked if out of range)\n";

    // Output: (blank line) Division and remainder (truncation and dangerous cases):
    std::cout << "\nDivision and remainder (truncation and dangerous cases):\n";

    // This performs integer division; the fractional part is discarded (truncation toward zero).
    int q1 = 7 / 3;
    // This computes the remainder corresponding to that truncated quotient.
    int r1 = 7 % 3;

    // Output: 7/3=2  7%3=1
    std::cout << "7/3=" << q1 << "  7%3=" << r1 << "\n";

    // This demonstrates truncation toward zero for negatives in modern C++.
    int q2 = -7 / 3;
    // This is the matching remainder.
    int r2 = -7 % 3;

    // Output: -7/3=-2  -7%3=-1
    std::cout << "-7/3=" << q2 << "  -7%3=" << r2 << "\n";

    // Output: (blank line) Conversions with bool (two-way interaction with integers):
    std::cout << "\nConversions with bool (two-way interaction with integers):\n";

    // This converts int to bool: zero becomes false, nonzero becomes true.
    bool b_from_int1 = 0;
    // This converts int to bool: any nonzero becomes true.
    bool b_from_int2 = -3;
    // This converts bool to int: false typically becomes 0.
    int int_from_bool1 = false;
    // This converts bool to int: true typically becomes 1.
    int int_from_bool2 = true;

    // Output: int->bool: 0=>0 -3=>1  bool->int: false=>0 true=>1
    std::cout << "int->bool: 0=>" << b_from_int1 << " -3=>" << b_from_int2
              << "  bool->int: false=>" << int_from_bool1 << " true=>" << int_from_bool2 << "\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // Output: (blank line) Undefined behavior examples (do not rely on outputs):
    std::cout << "\nUndefined behavior examples (do not rely on outputs):\n";

    // This sets a signed int to its maximum value.
    int ub_max = INT_MAX;
    // This causes signed overflow (undefined behavior), so the program may print any value or misbehave.
    int ub_overflow = ub_max + 1;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "INT_MAX + 1 => " << ub_overflow << " (undefined behavior)\n";

    // This sets a signed int to its minimum value.
    int ub_min = INT_MIN;
    // This attempts to negate INT_MIN; it overflows for typical two's-complement int ranges (undefined behavior).
    int ub_negate = -ub_min;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "-INT_MIN => " << ub_negate << " (undefined behavior)\n";

    // This divides INT_MIN by -1; the mathematical result is not representable in int (undefined behavior).
    int ub_div_overflow = ub_min / -1;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "INT_MIN / -1 => " << ub_div_overflow << " (undefined behavior)\n";

    // This creates a divisor of zero.
    int ub_zero = 0;
    // This divides by zero (undefined behavior), commonly crashing or producing no reliable output.
    int ub_div0 = 1 / ub_zero;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "1/0 => " << ub_div0 << " (undefined behavior)\n";

    // This demonstrates a bad overflow “check” pattern: the addition can overflow before the comparison runs (undefined behavior).
    int ub_a = INT_MAX;
    // This chooses an addend that would overflow.
    int ub_b = 1;
    // This expression itself can overflow (undefined behavior), so it is not a valid way to detect overflow.
    bool ub_bad_check = (ub_a + ub_b) >= ub_a;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "(INT_MAX+1 >= INT_MAX) => " << ub_bad_check << " (undefined behavior)\n";

    // This left-shifts a signed value into an unrepresentable range (undefined behavior).
    int ub_shift_signed = INT_MAX << 1;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "INT_MAX << 1 => " << ub_shift_signed << " (undefined behavior)\n";

    // This shift count is huge; it is guaranteed to be >= the bit-width of any practical unsigned int (undefined behavior).
    unsigned int ub_shift_count = 1u << INT_MAX;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "1u << INT_MAX => " << ub_shift_count << " (undefined behavior)\n";

    // This shift count is negative (undefined behavior).
    unsigned int ub_shift_negative = 1u << -1;
    // Output is not reliable because undefined behavior can change program behavior before printing.
    std::cout << "1u << -1 => " << ub_shift_negative << " (undefined behavior)\n";
#endif

#if SHOW_PLATFORM_DEPENDENT
    // Output: (blank line) Platform-dependent examples (results can vary):
    std::cout << "\nPlatform-dependent examples (results can vary):\n";

    // This converts -1 to unsigned int; the result is well-defined modulo 2^N (often becomes the max unsigned int).
    unsigned int u_from_neg = -1;
    // Output often: 4294967295 (if unsigned int is 32-bit), otherwise the max for that width.
    std::cout << "-1 to unsigned int => " << u_from_neg << " (wrap to max for that width)\n";

    // This converts a large unsigned to signed int; if not representable, the resulting int value is implementation-defined.
    int s_from_u = u_from_neg;
    // Output can vary across implementations/targets because the value may not be representable in int.
    std::cout << "unsigned max to int => " << s_from_u << " (implementation-defined if not representable)\n";

    // This shows that the width of long differs across common platforms (visible via LONG_MAX).
    // Output varies: LONG_MAX can match INT_MAX on some targets, or be much larger on others.
    std::cout << "LONG_MAX = " << LONG_MAX << " (platform-dependent long width)\n";
#endif

#if SHOW_COMPILE_ERRORS
    // This integer literal is too large for unsigned long long on typical implementations and should fail to compile.
    unsigned long long too_big_literal = 18446744073709551616ULL;

    // This attempts to create a negative shift count in a constant expression context; compilers may reject or error out.
    int bad_shift = 1 << -1;
#endif

    // This returns 0 to indicate success to the operating system.
    return 0;
}

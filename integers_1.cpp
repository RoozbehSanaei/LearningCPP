/* Contents:
   1) Integer families + ranges
   2) Signed vs unsigned meaning
   3) Conversions (safe vs unsafe)
   4) Checked add (good practice)
   5) Division/remainder hazards
   6) Shifts and bit masks (safe vs unsafe)
   7) Gated bad cases (compile errors / UB / platform-dependent)
*/

// Set to 1 to intentionally include code that does not compile.
#define SHOW_COMPILE_ERRORS 0
// Set to 1 to run undefined-behavior examples (dangerous).
#define SHOW_UNDEFINED_BEHAVIOR 0
// Set to 1 to show results that may differ across platforms/compilers.
#define SHOW_PLATFORM_DEPENDENT 0

// Gives std::cout for printing.
#include <iostream>
// Gives INT_MIN/INT_MAX/LONG_MAX/etc.
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

// Adds two int values only if the result fits in int.
bool safe_add_int(int a, int b, int& out)
{
    // Rejects positive overflow.
    if (b > 0 && a > INT_MAX - b) return false;
    // Rejects negative underflow.
    if (b < 0 && a < INT_MIN - b) return false;
    // Computes after checks.
    out = a + b;
    // Reports success.
    return true;
}

// Converts long long to int only if representable.
bool safe_ll_to_int(long long v, int& out)
{
    // Rejects below int range.
    if (v < (long long)INT_MIN) return false;
    // Rejects above int range.
    if (v > (long long)INT_MAX) return false;
    // Converts after checks.
    out = (int)v;
    // Reports success.
    return true;
}

// Shifts left safely: requires 0 <= shift < bit-width.
bool safe_lshift_uint(unsigned int v, int shift, unsigned int& out)
{
    // Rejects negative shift counts.
    if (shift < 0) return false;
    // Computes width.
    unsigned int w = uint_bits();
    // Rejects shift counts that are too large.
    if ((unsigned int)shift >= w) return false;
    // Shifts after checks (well-defined for unsigned).
    out = v << shift;
    // Reports success.
    return true;
}

// Program entry point.
int main()
{
    // Prints: heading.
    std::cout << "=== integer types: key behaviors and pitfalls ===\n";

    // Prints: ranges vary by platform.
    std::cout << "short: " << SHRT_MIN << " .. " << SHRT_MAX << "\n";
    // Prints: ranges vary by platform.
    std::cout << "int:   " << INT_MIN << " .. " << INT_MAX << "\n";
    // Prints: ranges vary by platform.
    std::cout << "long:  " << LONG_MIN << " .. " << LONG_MAX << "\n";
    // Prints: ranges vary by platform.
    std::cout << "ll:    " << LLONG_MIN << " .. " << LLONG_MAX << "\n";

    // Sets a signed value.
    int s = -1;
    // Sets an unsigned value.
    unsigned int u = 1u;
    // Compares signed and unsigned (signed may convert to unsigned).
    bool surprise = (s < u);
    // Prints: often "(-1 < 1u) => 0" (surprising result).
    std::cout << "(-1 < 1u) => " << surprise << "\n";

    // Demonstrates unsigned wraparound (defined modulo 2^N).
    unsigned int wrap = 0u;
    // Wraps to a large value (often UINT_MAX).
    --wrap;
    // Prints: often "wrap => 4294967295" (varies by width).
    std::cout << "unsigned --0 => " << wrap << "\n";

    // Prepares a too-large long long.
    long long big = 9000000000LL;
    // Holds a narrowed int.
    int narrowed = 0;
    // Checks if narrowing is safe.
    bool ok_narrow = safe_ll_to_int(big, narrowed);
    // Prints: "safe narrow ok => 0" on typical 32-bit int systems.
    std::cout << "safe narrow ok => " << ok_narrow << "\n";

#if SHOW_PLATFORM_DEPENDENT
    // Narrows without checking (result may vary if out of range).
    int unchecked = (int)big;
    // Prints: value is platform/implementation-dependent.
    std::cout << "unchecked narrow => " << unchecked << "\n";
#endif

    // Prepares an overflow case.
    int a = INT_MAX;
    // Adds 1 to it.
    int b = 1;
    // Holds checked result.
    int sum = 0;
    // Runs checked addition.
    bool ok_add = safe_add_int(a, b, sum);
    // Prints: "checked add ok => 0" (expected).
    std::cout << "checked add ok => " << ok_add << "\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // Signed overflow is undefined behavior.
    int ub_over = INT_MAX + 1;
    // Prints: unreliable result (do not trust).
    std::cout << "UB INT_MAX+1 => " << ub_over << "\n";
#endif

    // Prepares division inputs.
    int d1 = -7;
    // Prepares divisor.
    int d2 = 3;
    // Computes quotient (truncates toward zero).
    int q = d1 / d2;
    // Computes remainder (sign follows dividend).
    int r = d1 % d2;
    // Prints: typically "q=-2 r=-1".
    std::cout << "-7/3 => q=" << q << " r=" << r << "\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // INT_MIN / -1 can be undefined behavior if not representable.
    int ub_div = INT_MIN / -1;
    // Prints: unreliable (may trap on some systems).
    std::cout << "UB INT_MIN/-1 => " << ub_div << "\n";
#endif

    // Prepares a value to shift.
    unsigned int one = 1u;
    // Prepares shift amount.
    int sh = 3;
    // Holds shift result.
    unsigned int sh_out = 0u;
    // Runs safe shift.
    bool ok_sh = safe_lshift_uint(one, sh, sh_out);
    // Prints: "ok=1 out=8" (expected).
    std::cout << "safe shift ok=" << ok_sh << " out=" << sh_out << "\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // Shifting by a negative count is undefined behavior.
    unsigned int ub_sh1 = 1u << -1;
    // Prints: unreliable (do not trust).
    std::cout << "UB 1u<<-1 => " << ub_sh1 << "\n";
#endif

#if SHOW_UNDEFINED_BEHAVIOR
    // Shifting by >= width is undefined behavior.
    unsigned int ub_sh2 = 1u << (int)uint_bits();
    // Prints: unreliable (do not trust).
    std::cout << "UB 1u<<width => " << ub_sh2 << "\n";
#endif

#if SHOW_COMPILE_ERRORS
    // Intentional syntax error to demonstrate a gated compile failure.
    int this_wont_compile = ;
#endif

    // Returns success.
    return 0;
}

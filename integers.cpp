// Concepts covered in this file (high-level checklist):
// 1) The built-in integer types: short, int, long, long long (and their unsigned forms).
// 2) Portability: type widths differ by platform; ranges are not fixed.
// 3) Signed vs unsigned meaning and when unsigned is appropriate (often for bit patterns).
// 4) Overflow/underflow: unsigned wraparound is defined; signed overflow is undefined behavior.
// 5) Mixed signed/unsigned arithmetic and comparisons can yield surprising results.
// 6) Integer division truncation and remainder behavior (including negatives).
// 7) Conversions between integer types: narrowing can be implementation-defined when out of range.
// 8) Minimal two-way relationship needed here: bool <-> integer values.

// This header provides std::cout for printing results (printing is used only to observe behavior).
#include <iostream>
// This header provides integer range macros like INT_MAX, LONG_MIN, ULLONG_MAX.
#include <climits>

// This controls code that intentionally does not compile (to demonstrate compile-time mistakes).
#define SHOW_COMPILE_ERRORS 0
// This controls code that can execute with undefined behavior (to demonstrate “do not do this” cases).
#define SHOW_UNDEFINED_BEHAVIOR 0
// This controls code whose output can vary by platform/compiler (to demonstrate portability pitfalls).
#define SHOW_PLATFORM_DEPENDENT 1

// This function adds two int values with a pre-check to avoid signed overflow.
bool safe_add_int(int a, int b, int& out_sum)
{
    // This checks the “positive overflow” case (a + b would exceed INT_MAX).
    if (b > 0 && a > INT_MAX - b)
    {
        // This reports failure and avoids performing undefined behavior.
        return false;
    }

    // This checks the “negative underflow” case (a + b would go below INT_MIN).
    if (b < 0 && a < INT_MIN - b)
    {
        // This reports failure and avoids performing undefined behavior.
        return false;
    }

    // This computes the sum only after the range checks passed.
    out_sum = a + b;

    // This reports success.
    return true;
}

// This function adds two unsigned int values and detects wraparound before it happens.
bool safe_add_uint(unsigned int a, unsigned int b, unsigned int& out_sum)
{
    // This checks whether a + b would exceed UINT_MAX (even though wrap is defined, we treat it as a bug here).
    if (b > UINT_MAX - a)
    {
        // This reports failure and avoids wraparound.
        return false;
    }

    // This computes the sum only after the range check passed.
    out_sum = a + b;

    // This reports success.
    return true;
}

// This is where execution begins.
int main()
{
    std::cout << "Integer types demo (focus: pitfalls + practices)\n\n";

    // This prints the representable ranges for the signed built-in integer types.
    std::cout << "signed short range: [" << SHRT_MIN << ", " << SHRT_MAX << "]\n";
    std::cout << "signed int range:   [" << INT_MIN << ", " << INT_MAX << "]\n";
    std::cout << "signed long range:  [" << LONG_MIN << ", " << LONG_MAX << "]\n";
    std::cout << "signed long long range: [" << LLONG_MIN << ", " << LLONG_MAX << "]\n\n";

    // This prints the representable ranges for the unsigned built-in integer types (minimum is always 0).
    std::cout << "unsigned short range: [0, " << USHRT_MAX << "]\n";
    std::cout << "unsigned int range:   [0, " << UINT_MAX << "]\n";
    std::cout << "unsigned long range:  [0, " << ULONG_MAX << "]\n";
    std::cout << "unsigned long long range: [0, " << ULLONG_MAX << "]\n\n";

    // This creates two signed ints for a normal arithmetic example.
    int x = 42;
    // This creates a negative signed int.
    int y = -7;

    std::cout << "x=" << x << " y=" << y << "\n\n";

    // This demonstrates the two-way relationship with bool: zero becomes false.
    bool b_from_zero = 0;
    // This demonstrates the two-way relationship with bool: any nonzero becomes true.
    bool b_from_nonzero = x;
    // This demonstrates the reverse direction: true becomes 1 when stored in an int.
    int i_from_true = true;
    // This demonstrates the reverse direction: false becomes 0 when stored in an int.
    int i_from_false = false;

    std::cout << "int->bool: 0 => " << b_from_zero << " ; 42 => " << b_from_nonzero << "\n";
    std::cout << "bool->int: true => " << i_from_true << " ; false => " << i_from_false << "\n\n";

    // This prepares an output variable for checked signed addition.
    int checked_sum = 0;
    // This performs a checked signed addition that avoids undefined behavior.
    bool add_ok = safe_add_int(INT_MAX, 1, checked_sum);

    std::cout << "checked signed add: INT_MAX + 1 succeeded? " << add_ok << " (should be false)\n\n";

    // This prepares two unsigned ints for a wraparound demonstration.
    unsigned int u0 = 0u;
    // This subtracts 1 from 0 in unsigned arithmetic; this wraps to a large value (often UINT_MAX).
    unsigned int wrapped = u0 - 1u;

    std::cout << "unsigned wraparound: 0u - 1u => " << wrapped << " (often UINT_MAX; surprising if you expected -1)\n\n";

    // This prepares an output variable for checked unsigned addition.
    unsigned int checked_usum = 0u;
    // This performs a checked unsigned addition that avoids wraparound.
    bool uadd_ok = safe_add_uint(UINT_MAX, 1u, checked_usum);

    std::cout << "checked unsigned add: UINT_MAX + 1 succeeded? " << uadd_ok << " (should be false)\n\n";

    // This creates a signed negative value.
    int sneg = -1;
    // This creates a small unsigned value.
    unsigned int upos = 1u;
    // This comparison can be surprising because the signed value may be converted to unsigned before comparing.
    bool mixed_compare = (sneg < upos);

    std::cout << "mixed comparison: (-1 < 1u) => " << mixed_compare << " (often surprising: frequently false)\n\n";

    // This creates two short values (a smaller integer type).
    short s1 = 30000;
    // This creates another short value.
    short s2 = 10000;
    // This adds them; the operation is typically performed in int after promotion.
    int promoted_sum = s1 + s2;
    // This converts the promoted result back into short; if out of range, the stored value can be unexpected.
    short narrowed_back = static_cast<short>(promoted_sum);

    std::cout << "promotion: short+short computed (typically) as int => " << promoted_sum << "\n";
    std::cout << "narrowing back to short may change the value: " << narrowed_back << " (unexpected if it does not fit)\n\n";

    // This demonstrates integer division truncation toward zero.
    int q1 = -7 / 3;
    // This demonstrates remainder consistent with the division rule.
    int r1 = -7 % 3;

    std::cout << "division: -7/3 => " << q1 << " ; -7%3 => " << r1 << " (often surprising if you expected floor division)\n\n";

#if SHOW_PLATFORM_DEPENDENT
    // This assigns a value that may not fit in int on common platforms; if it does not fit, the result is implementation-defined.
    int impl_defined_narrow = 5000000000LL;

    std::cout << "platform-dependent narrowing: int x = 5000000000LL gives " << impl_defined_narrow
              << " (implementation-defined if the value does not fit)\n\n";

    // This shows that long differs by platform by printing its maximum (often ~2e9 on LLP64, ~9e18 on LP64).
    std::cout << "LONG_MAX=" << LONG_MAX << " (indicates long width on this platform)\n\n";
#endif

#if SHOW_UNDEFINED_BEHAVIOR
    // This creates a value at the maximum representable int.
    int m = INT_MAX;
    // This performs signed overflow; this is undefined behavior (the program’s behavior is not reliable).
    int ub_overflow = m + 1;

    std::cout << "UNDEFINED: INT_MAX + 1 => " << ub_overflow << " (output is not reliable)\n\n";

    // This shifts into the sign bit on typical 32-bit int; this can be undefined if the result is not representable.
    int ub_shift = 1 << 31;

    std::cout << "UNDEFINED (on many platforms): 1<<31 => " << ub_shift << " (output is not reliable)\n\n";

    // This divides by zero; this is undefined behavior.
    int ub_div0 = 1 / 0;

    std::cout << "UNDEFINED: 1/0 => " << ub_div0 << " (output is not reliable)\n\n";
#endif

#if SHOW_COMPILE_ERRORS
    // This is ill-formed: braced initialization rejects converting a negative value to an unsigned type.
    unsigned int bad_unsigned{-1};

    // This is ill-formed: braced initialization rejects converting a negative value to an unsigned short.
    unsigned short bad_ushort{-1};
#endif

    // This returns 0 to indicate the program ended successfully.
    return 0;
}

// Concepts covered in this file (high-level checklist):
// 1) What “character types” are: code units stored in integer-like types (not “text strings”).
// 2) The narrow character family: char, signed char, unsigned char.
// 3) The wide/Unicode code-unit types: wchar_t, char8_t (if available), char16_t, char32_t.
// 4) Plain char signedness and range variability across platforms.
// 5) Safe vs unsafe use of <cctype> classification (std::isalpha) with char values.
// 6) Character literal pitfalls: hex escape greediness, multi-character literals.
// 7) Narrowing / out-of-range assignments to character types and why results can surprise.
// 8) Minimal two-way relationships needed for this topic: bool <-> character types.

// This header provides std::cout for printing results (printing is used only to observe behavior).
#include <iostream>
// This header provides std::isalpha and related character classification functions.
#include <cctype>
// This header provides CHAR_BIT, CHAR_MIN, CHAR_MAX, UCHAR_MAX, SCHAR_MIN, SCHAR_MAX.
#include <climits>
// This header provides WCHAR_MIN and WCHAR_MAX for wchar_t bounds.
#include <cwchar>

// This controls code that intentionally does not compile (to demonstrate compile-time mistakes).
#define SHOW_COMPILE_ERRORS 0
// This controls code that can execute with undefined behavior (to demonstrate “do not do this” cases).
#define SHOW_UNDEFINED_BEHAVIOR 0
// This controls code whose output can vary by platform, compiler, locale, or language mode.
#define SHOW_PLATFORM_DEPENDENT 1

// This helper prints the numeric value of an unsigned char as a normal number (not as a glyph).
void print_u8_value(unsigned char uc)
{
    // This converts the code unit to an unsigned int so stream output is numeric.
    unsigned int n = static_cast<unsigned int>(uc);

    std::cout << n;
}

// This helper performs a safe std::isalpha call for a value that originated as a char.
bool is_alpha_safely(char c)
{
    // This converts to unsigned char first so the value is guaranteed to be 0..UCHAR_MAX.
    unsigned char uc = static_cast<unsigned char>(c);
    // This converts to int because <cctype> functions take int, and require unsigned-char range or EOF.
    int as_int = static_cast<int>(uc);
    // This calls std::isalpha with a defined argument range; the returned nonzero value means “true”.
    return std::isalpha(as_int) != 0;
}

// This is where execution begins.
int main()
{
    // This prints a banner so outputs are easier to interpret.
    std::cout << "Character types demo (focus: pitfalls + practices)\n\n";

    // This prints CHAR_BIT, the number of bits in a byte on this platform (often 8, but not guaranteed).
    std::cout << "CHAR_BIT=" << CHAR_BIT << "\n\n";

    // This creates an ordinary character literal of type char (narrow character type).
    char a = 'A';
    // This converts char to unsigned char so the numeric code unit is observed consistently.
    unsigned char a_uc = static_cast<unsigned char>(a);

    std::cout << "plain char a='A' as numeric code unit="; std::cout << "";  // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption
    std::cout << "";                                                        // print line exemption

    // This prints the numeric value using the helper (usually 65 in ASCII-like encodings).
    std::cout << "plain char a='A' as numeric code unit="; print_u8_value(a_uc); std::cout << "\n\n";

    // This shows that plain char may be signed or unsigned depending on platform (range varies).
    std::cout << "plain char range: [" << CHAR_MIN << ", " << CHAR_MAX << "] (platform-dependent)\n";
    // This shows the explicit signed-char range (always signed).
    std::cout << "signed char range: [" << SCHAR_MIN << ", " << SCHAR_MAX << "]\n";
    // This shows the explicit unsigned-char max (minimum is 0).
    std::cout << "unsigned char range: [0, " << UCHAR_MAX << "]\n\n";

    // This creates a signed char explicitly (use this type when you truly want signed behavior).
    signed char sc = static_cast<signed char>(-1);
    // This creates an unsigned char explicitly (use this type when you truly want 0..UCHAR_MAX behavior).
    unsigned char uc = static_cast<unsigned char>(255u);

    std::cout << "signed char -1 printed as int=" << static_cast<int>(sc) << "\n";
    std::cout << "unsigned char 255 printed as int=" << static_cast<int>(uc) << "\n\n";

    // This demonstrates a common surprise: streaming an unsigned char often prints a glyph, not a number (unexpected if you expected 65).
    unsigned char glyph_like = static_cast<unsigned char>(65u);

    std::cout << "streaming unsigned char 65 may print as glyph 'A': " << glyph_like << "\n";
    std::cout << "printing numeric value instead: "; print_u8_value(glyph_like); std::cout << "\n\n";

    // This demonstrates the two-way relationship with bool: a nonzero char becomes true.
    bool b_from_char_nonzero = a;
    // This demonstrates the two-way relationship with bool: a zero char becomes false.
    bool b_from_char_zero = static_cast<char>(0);
    // This demonstrates the reverse direction: true becomes 1 when converted to char (often a non-printable control if printed as a glyph).
    char char_from_true = static_cast<char>(true);
    // This demonstrates the reverse direction: false becomes 0 (the null character).
    char char_from_false = static_cast<char>(false);
    // This converts the result to unsigned char to display the stored code unit as a number.
    unsigned char t_uc = static_cast<unsigned char>(char_from_true);
    // This converts the result to unsigned char to display the stored code unit as a number.
    unsigned char f_uc = static_cast<unsigned char>(char_from_false);

    std::cout << "char->bool: 'A' => " << b_from_char_nonzero << " ; '\\0' => " << b_from_char_zero << "\n";
    std::cout << "bool->char numeric: true => "; print_u8_value(t_uc); std::cout << " ; false => "; print_u8_value(f_uc); std::cout << "\n\n";

#if SHOW_PLATFORM_DEPENDENT
    // This stores the byte 0xFF into a plain char; if plain char is signed, this value can be negative (unexpected if you assumed 0..255).
    char c_ff = '\xFF';
    // This converts to unsigned char so we can observe the code unit in 0..UCHAR_MAX regardless of plain char signedness.
    unsigned char c_ff_uc = static_cast<unsigned char>(c_ff);

    std::cout << "plain char '\\xFF' observed code unit as number="; print_u8_value(c_ff_uc); std::cout << "\n";
    std::cout << "If plain char is signed, using it as a small integer can surprise (it may behave like a negative value).\n\n";
#endif

    // This demonstrates a character-literal pitfall: \\x escape sequences keep consuming following hex digits (unexpected if you assumed exactly two digits).
    // This line often does NOT mean '\\x41' followed by 'B'; instead it tries to form one hex value from all hex digits that follow.
    char hex_greedy = '\x41B';
    // This converts to unsigned char so the stored code unit is shown consistently.
    unsigned char hex_greedy_uc = static_cast<unsigned char>(hex_greedy);

    std::cout << "hex escape greediness: '\\x41B' stored as one code unit, observed="; print_u8_value(hex_greedy_uc);
    std::cout << " (unexpected if you expected 'A' then 'B')\n\n";

    // This prepares a byte that may be negative if plain char is signed (0xE9 is 233, which can map to -23 in signed 8-bit char).
    char maybe_negative = '\xE9';
    // This calls the safe wrapper that converts through unsigned char before calling std::isalpha (avoids undefined behavior).
    bool alpha_safe = is_alpha_safely(maybe_negative);

    std::cout << "safe std::isalpha on '\\xE9' => " << alpha_safe << " (result depends on locale/execution encoding)\n\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // This uses the same value but calls std::isalpha directly with a possibly-negative char.
    // This is undefined behavior when the char value is not EOF and not representable as unsigned char.
    bool alpha_ub = std::isalpha(maybe_negative) != 0;

    std::cout << "UNDEFINED BEHAVIOR path: std::isalpha(maybe_negative) => " << alpha_ub << " (output is not reliable)\n\n";
#endif

#if SHOW_PLATFORM_DEPENDENT
    // This demonstrates that wchar_t exists for “wide” character literals, but its width and encoding are platform-dependent.
    wchar_t wz = L'Z';
    // This reads WCHAR_MIN for the platform.
    long wmin = static_cast<long>(WCHAR_MIN);
    // This reads WCHAR_MAX for the platform.
    long wmax = static_cast<long>(WCHAR_MAX);

    std::cout << "wchar_t value of L'Z' as number=" << static_cast<unsigned long>(wz) << "\n";
    std::cout << "wchar_t range: [" << wmin << ", " << wmax << "] (platform-dependent)\n\n";
#endif

#if defined(__cpp_char8_t)
    // This demonstrates char8_t in C++20+: it is a distinct type intended to hold UTF-8 code units.
    char8_t u8a = u8'A';
    // This converts char8_t to an unsigned int for numeric display.
    unsigned int u8a_num = static_cast<unsigned int>(u8a);

    std::cout << "char8_t u8'A' numeric code unit=" << u8a_num << "\n\n";
#endif

    // This demonstrates char16_t: intended to hold UTF-16 code units.
    char16_t u16a = u'A';
    // This converts char16_t to an unsigned int for numeric display.
    unsigned int u16a_num = static_cast<unsigned int>(u16a);

    std::cout << "char16_t u'A' numeric code unit=" << u16a_num << "\n";

    // This demonstrates char32_t: intended to hold UTF-32 code units.
    char32_t u32a = U'A';
    // This converts char32_t to an unsigned int for numeric display.
    unsigned int u32a_num = static_cast<unsigned int>(u32a);

    std::cout << "char32_t U'A' numeric code unit=" << u32a_num << "\n\n";

#if SHOW_PLATFORM_DEPENDENT
    // This demonstrates a “narrowing-style” surprise: assigning 300 to an unsigned char typically keeps only the low bits (often 44 if CHAR_BIT==8).
    unsigned char narrow_uc = static_cast<unsigned char>(300u);
    // This prints the observed numeric result (unexpected if you expected 300).
    std::cout << "narrowing to unsigned char: 300 becomes "; print_u8_value(narrow_uc);
    std::cout << " (unexpected if you expected 300)\n\n";
#endif

#if SHOW_PLATFORM_DEPENDENT
    // This demonstrates a multi-character literal; it is conditionally-supported, has an implementation-defined value, and is not a single “character”.
    // This uses an unsigned long for printing the value without teaching “integer types” as a topic.
    unsigned long multi_val = static_cast<unsigned long>('AB');

    std::cout << "multi-character literal 'AB' numeric value=" << multi_val << " (implementation-defined and not portable)\n\n";
#endif

#if SHOW_COMPILE_ERRORS
    // This attempts to create an object of type void, which is not allowed.
    void bad_void_object = 0;

    // This assigns nullptr to a character type, which is not allowed without an explicit conversion.
    char bad_from_nullptr = nullptr;

    // This attempts to store a code point requiring a surrogate pair into a single char16_t code unit, which is ill-formed.
    char16_t bad_u16 = u'\U0001F600';

#if defined(__cpp_char8_t)
    // This attempts to create a single char8_t character literal that is not representable in one UTF-8 code unit, which is ill-formed.
    char8_t bad_u8 = u8'€';
#endif
#endif

    // This returns 0 to indicate the program ended successfully.
    return 0;
}

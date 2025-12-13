/*
Contents
1) Character types at a glance
2) plain char vs signed char vs unsigned char
3) Conversions between character types and bool (both directions)
4) Observing numeric code units safely
5) Portability: plain char signedness and platform-dependent results
6) Escape-sequence pitfalls in character literals (notably \x “greediness”)
7) <cctype> classification pitfalls (std::isalpha) and the safe pattern
8) wchar_t, char8_t (if available), char16_t, char32_t demonstrations
9) Intentionally-bad code blocks (compile errors / undefined behavior / platform-dependent)
*/

// This header enables printing to the terminal with std::cout.
#include <iostream>
// This header provides character classification functions like std::isalpha.
#include <cctype>
// This header provides portable limits like CHAR_MIN, CHAR_MAX, UCHAR_MAX.
#include <climits>
// This header provides wide-character limits like WCHAR_MIN, WCHAR_MAX.
#include <cwchar>
// This header provides std::nullptr_t.
#include <cstddef>

// This macro controls blocks that intentionally do not compile (set to 1 to see the compiler errors).
#define SHOW_COMPILE_ERRORS 0
// This macro controls blocks that may trigger undefined behavior at runtime (set to 1 only for demonstration).
#define SHOW_UNDEFINED_BEHAVIOR 0
// This macro controls blocks whose outputs vary across platforms/compilers (set to 1 to observe portability issues).
#define SHOW_PLATFORM_DEPENDENT 1

// This function converts an unsigned char code unit to a printable number (0..UCHAR_MAX on typical systems).
unsigned int code_unit_number(unsigned char c)
{
    // This conversion ensures we print a number instead of printing a character glyph.
    return static_cast<unsigned int>(c);
}

// This function demonstrates the safe way to call std::isalpha for a value stored in a char.
bool is_alpha_safe(char c)
{
    // This converts the possibly-signed char value into an unsigned char code unit (0..UCHAR_MAX).
    unsigned char uc = static_cast<unsigned char>(c);
    // This passes an int value that is guaranteed to be representable as unsigned char (required by std::isalpha’s contract).
    int arg = static_cast<int>(uc);
    // This calls std::isalpha and returns true if it reports “alphabetic” in the current C locale.
    return std::isalpha(arg) != 0;
}

// This is where program execution starts.
int main()
{
    // This creates a plain char holding the character literal 'A' (a single code unit in the execution character set).
    char plain = 'A';
    // This converts plain char to unsigned char so we can observe its numeric code unit consistently.
    unsigned char plain_uc = static_cast<unsigned char>(plain);
    // This converts the code unit into an unsigned int so printing shows a number instead of a glyph.
    unsigned int plain_num = code_unit_number(plain_uc);

    // Output: prints the glyph 'A' and its numeric code unit (commonly 65 on ASCII-like systems; numeric value is implementation-dependent).
    std::cout << "plain char '" << plain << "' numeric=" << plain_num << "\n";

    // This creates an explicitly signed character type (its range is at least -127..127, often -128..127).
    signed char sc = -1;
    // This creates an explicitly unsigned character type (its range is 0..UCHAR_MAX, often 0..255).
    unsigned char uc = 255u;

    // Output: prints -1 (or another negative value if you changed sc).
    std::cout << "signed char as number=" << static_cast<int>(sc) << "\n";
    // Output: prints 255 (or another 0..UCHAR_MAX value if you changed uc).
    std::cout << "unsigned char as number=" << static_cast<int>(uc) << "\n";

    // This demonstrates a common surprise: streaming an unsigned char often prints a glyph, not a number (unexpected if you expected “65”).
    unsigned char glyph_surprise = 65u;
    // Output: often prints "A" (a glyph), not "65" (unexpected if you intended numeric output).
    std::cout << "streaming unsigned char 65 as a character: " << glyph_surprise << "\n";
    // Output: prints 65 as a number.
    std::cout << "streaming its numeric code unit instead: " << code_unit_number(glyph_surprise) << "\n";

    // This shows the two-way relationship with bool: a nonzero char converts to true.
    bool b_from_plain = plain;
    // This creates the “null character” (a char with numeric value 0).
    char zero_char = '\0';
    // This shows the two-way relationship with bool: a zero char converts to false.
    bool b_from_zero = zero_char;

    // Output: prints 1 then 0 (typical formatting is 1/0 unless you enable boolalpha formatting).
    std::cout << "char->bool: 'A' => " << b_from_plain << ", '\\0' => " << b_from_zero << "\n";

    // This demonstrates the reverse direction: true converts to a char with numeric value 1 (often a non-printable control).
    char char_from_true = static_cast<char>(true);
    // This demonstrates the reverse direction: false converts to a char with numeric value 0 (the null character).
    char char_from_false = static_cast<char>(false);

    // Output: prints numeric values (typically 1 and 0).
    std::cout << "bool->char numeric: true => " << code_unit_number(static_cast<unsigned char>(char_from_true))
              << ", false => " << code_unit_number(static_cast<unsigned char>(char_from_false)) << "\n";

    // This records the minimum value representable by plain char on this platform (platform-dependent).
    int char_min = CHAR_MIN;
    // This records the maximum value representable by plain char on this platform (platform-dependent).
    int char_max = CHAR_MAX;
    // This records the maximum value representable by unsigned char (often 255).
    int uchar_max = UCHAR_MAX;

    // Output: prints the char range; possible outputs include [-128,127] or [0,255] (and other ranges on unusual targets).
    std::cout << "plain char range: [" << char_min << ", " << char_max << "], UCHAR_MAX=" << uchar_max << "\n";

#if SHOW_PLATFORM_DEPENDENT
    // This stores hex 0xFF into a plain char; if plain char is signed, reading it as int may produce -1 (unexpected if you assumed 255).
    char c_ff = '\xFF';
    // This converts it to unsigned char to observe the stored code unit as a value in 0..UCHAR_MAX.
    unsigned char c_ff_uc = static_cast<unsigned char>(c_ff);
    // This converts plain char directly to int; if char is signed and the value is 0xFF, this is often -1 (platform-dependent).
    int c_ff_as_int = c_ff;

    // Output: prints code unit 255 via the unsigned-char path; prints either -1 or 255 via the plain-char-to-int path (platform-dependent).
    std::cout << "'\\xFF' as unsigned-char code unit=" << code_unit_number(c_ff_uc)
              << ", as int via plain char=" << c_ff_as_int
              << " (often -1 if plain char is signed)\n";

    // This demonstrates wchar_t with a wide character literal; wchar_t’s size/range is platform-dependent.
    wchar_t w = L'Z';
    // This captures the maximum wchar_t value as a wide integer macro (platform-dependent).
    unsigned long wmax = static_cast<unsigned long>(WCHAR_MAX);

    // Output: prints the numeric value of L'Z' (typically 90) and WCHAR_MAX (platform-dependent).
    std::cout << "wchar_t: L'Z' numeric=" << static_cast<unsigned long>(w) << ", WCHAR_MAX=" << wmax << "\n";
#endif

    // This demonstrates an escape-sequence pitfall: after \x, the compiler keeps consuming hex digits (unexpected if you assumed it stops after two).
    char greedy_hex = '\x41B';
    // This converts to unsigned char to observe the stored code unit after truncation/adjustment to fit in char.
    unsigned char greedy_hex_uc = static_cast<unsigned char>(greedy_hex);

    // Output: if char is 8-bit, many compilers effectively interpret \x41B as 0x41B then store the low 8 bits (often 0x1B = 27); exact behavior depends on char width and implementation details.
    std::cout << "greedy hex '\\x41B' stored code unit=" << code_unit_number(greedy_hex_uc)
              << " (often 27 on 8-bit char targets)\n";

    // This demonstrates a safer call to std::isalpha when the char might hold a non-ASCII code unit.
    char maybe_non_ascii = '\xE9';
    // This calls the safe wrapper that converts to unsigned char before classification.
    bool alpha_safe = is_alpha_safe(maybe_non_ascii);

    // Output: prints 0 or 1 depending on the current C locale and how that locale classifies the code unit.
    std::cout << "safe std::isalpha on '\\xE9' => " << alpha_safe << " (depends on locale/encoding)\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // This stores a value that may be negative when plain char is signed.
    char ub_char = '\xE9';
    // This passes a possibly-negative char directly to std::isalpha; this is undefined behavior unless the value equals EOF or is representable as unsigned char.
    bool ub_alpha = std::isalpha(ub_char) != 0;

    // Output: could print 0 or 1, or behave unpredictably; the result is not reliable because behavior is undefined.
    std::cout << "UNDEFINED: std::isalpha(plain char '\\xE9') => " << ub_alpha << " (not reliable)\n";
#endif

#if defined(__cpp_char8_t)
    // This creates a UTF-8 code unit literal of type char8_t (available in C++20 and later).
    char8_t u8a = u8'A';
    // This converts to an unsigned number for printing because char8_t is a distinct type and is not streamed like plain char.
    unsigned int u8a_num = static_cast<unsigned int>(u8a);

    // Output: prints 65 for u8'A'.
    std::cout << "char8_t u8'A' code unit=" << u8a_num << "\n";
#endif

    // This creates a UTF-16 code unit literal of type char16_t (it holds a 16-bit code unit value).
    char16_t u16a = u'A';
    // This converts the code unit to an unsigned number for printing.
    unsigned int u16a_num = static_cast<unsigned int>(u16a);

    // Output: prints 65 for u'A'.
    std::cout << "char16_t u'A' code unit=" << u16a_num << "\n";

    // This creates a UTF-32 code unit literal of type char32_t (it holds a 32-bit code unit value).
    char32_t u32a = U'A';
    // This converts the code unit to an unsigned number for printing.
    unsigned int u32a_num = static_cast<unsigned int>(u32a);

    // Output: prints 65 for U'A'.
    std::cout << "char32_t U'A' code unit=" << u32a_num << "\n";

#if SHOW_PLATFORM_DEPENDENT
    // This is a multi-character literal; it has type int and its value is implementation-defined (unexpected if you assumed it is a “char”).
    int multichar = 'AB';

    // Output: prints some integer value that varies by implementation (commonly related to packing 'A' and 'B' into an int).
    std::cout << "multi-character literal 'AB' as int=" << multichar << " (implementation-defined)\n";
#endif

    // This demonstrates a narrowing conversion into plain char; the result is implementation-defined if the value does not fit (unexpected if you assumed it “just works”).
    int large_number = 300;
    // This stores 300 into a plain char; if char cannot represent 300, the stored value is implementation-defined.
    char narrowed = large_number;
    // This converts to unsigned char to observe the stored code unit as 0..UCHAR_MAX.
    unsigned char narrowed_uc = static_cast<unsigned char>(narrowed);

    // Output: prints a value that may differ from 300 (commonly 44 on 8-bit unsigned-char targets because 300 mod 256 = 44); exact result is implementation-defined for plain char narrowing.
    std::cout << "narrowing int 300 -> char observed code unit=" << code_unit_number(narrowed_uc)
              << " (often 44 on 8-bit unsigned-char targets)\n";

#if SHOW_COMPILE_ERRORS
    // This attempts to declare a variable of type void; this is ill-formed and must not compile.
    void bad_void = 0;

    // This attempts to assign nullptr to a character type; this is ill-formed without an explicit conversion.
    char bad_from_nullptr = nullptr;

    // This shows std::nullptr_t as its own type (this line compiles, but is included here so you see the type directly).
    std::nullptr_t np = nullptr;

    // This attempts to assign a wide character literal to plain char; this is ill-formed if it does not fit in char.
    char bad_wide_to_char = L'Z';

#if defined(__cpp_char8_t)
    // This attempts to store a non-single-code-unit character literal into char8_t; compilers commonly reject this as ill-formed.
    char8_t bad_u8 = u8'€';
#endif

    // This attempts to store a code point requiring multiple UTF-16 code units into a single char16_t; this is ill-formed.
    char16_t bad_u16 = u'\U0001F600';
#endif

    // This returns 0 to indicate successful completion.
    return 0;
}

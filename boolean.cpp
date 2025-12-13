// Concepts covered in this file (high-level checklist):
// 1) The two values of bool: false and true.
// 2) Conversions into bool (zero -> false, nonzero -> true) and out of bool (false -> 0, true -> 1).
// 3) Normalizing “truthy” values (e.g., using !!) and when it helps readability.
// 4) Short-circuit evaluation of && and ||, and why side effects can be skipped.
// 5) A common bug: assignment inside a condition (using = instead of ==).
// 6) Another common bug: using & and | instead of && and || with bool (both sides evaluate).
// 7) Undefined behavior example: using an uninitialized bool (gated).
// 8) Controlled “bad examples”: compile errors and unsafe behavior behind #if switches.

// This header provides std::cout for printing observable outcomes.
#include <iostream>

// This switch enables code that intentionally does not compile (to demonstrate compile-time mistakes).
#define SHOW_COMPILE_ERRORS 0
// This switch enables code that can execute with undefined behavior (to demonstrate unsafe mistakes).
#define SHOW_UNDEFINED_BEHAVIOR 0
// This switch enables notes/demos where behavior depends on evaluation strategy, side effects, or environment.
#define SHOW_PLATFORM_DEPENDENT 1

// This global counter tracks whether a function was actually called (used to show short-circuit behavior).
int g_calls = 0;

// This function returns the bool value you pass in, but also increments a counter as a visible side effect.
bool side_effect(bool value)
{
    // This increments the global counter each time the function runs.
    g_calls = g_calls + 1;
    // This returns the input value unchanged.
    return value;
}

// This function resets the global call counter to zero.
void reset_calls()
{
    // This sets the global counter back to 0 before a new demonstration.
    g_calls = 0;
}

// This is where execution begins.
int main()
{
    // This creates a bool with the value true.
    bool t = true;
    // This creates a bool with the value false.
    bool f = false;

    std::cout << "Basic bool values: true=" << t << " false=" << f << "\n\n";

    // This converts a zero number to bool; the result is false.
    bool from_zero = 0;
    // This converts a nonzero number to bool; the result is true.
    bool from_two = 2;

    std::cout << "Conversions into bool: 0->" << from_zero << " 2->" << from_two
              << " (unexpected if you expected 2 to stay '2')\n";

    // This converts bool to int; false becomes 0 (guaranteed).
    int int_from_false = f;
    // This converts bool to int; true becomes 1 (guaranteed).
    int int_from_true = t;

    std::cout << "Conversions out of bool (to int): false->" << int_from_false << " true->" << int_from_true << "\n\n";

    // This creates a number that is intended to be treated as a “truthy/falsey” value.
    int maybe_truthy = 5;
    // This converts to bool (nonzero -> true), then back to int (true -> 1), normalizing to 0/1.
    int normalized_01 = !!maybe_truthy;

    std::cout << "Normalization (!!): 5 becomes " << normalized_01 << " (unexpected if you expected 5)\n\n";

    // This resets the call counter before demonstrating && short-circuit behavior.
    reset_calls();
    // This expression uses &&; because the left side is false, the right side is not evaluated.
    bool sc_and = false && side_effect(true);
    // This snapshots how many times side_effect() ran (it should be 0 here).
    int calls_after_and = g_calls;

    std::cout << "Short-circuit &&: result=" << sc_and << " calls=" << calls_after_and
              << " (unexpected if you expected the right side to run)\n";

    // This resets the call counter before demonstrating || short-circuit behavior.
    reset_calls();
    // This expression uses ||; because the left side is true, the right side is not evaluated.
    bool sc_or = true || side_effect(false);
    // This snapshots how many times side_effect() ran (it should be 0 here).
    int calls_after_or = g_calls;

    std::cout << "Short-circuit ||: result=" << sc_or << " calls=" << calls_after_or
              << " (unexpected if you expected the right side to run)\n\n";

    // This resets the call counter before demonstrating the & operator with bool.
    reset_calls();
    // This uses & with bool; both sides are evaluated, so side_effect() runs even though left is false.
    bool eager_and = false & side_effect(true);
    // This snapshots how many times side_effect() ran (it should be 1 here).
    int calls_after_eager_and = g_calls;

    std::cout << "Using '&' with bool: result=" << eager_and << " calls=" << calls_after_eager_and
              << " (unexpected if you expected short-circuit)\n";

    // This resets the call counter before demonstrating the | operator with bool.
    reset_calls();
    // This uses | with bool; both sides are evaluated, so side_effect() runs even though left is true.
    bool eager_or = true | side_effect(false);
    // This snapshots how many times side_effect() ran (it should be 1 here).
    int calls_after_eager_or = g_calls;

    std::cout << "Using '|' with bool: result=" << eager_or << " calls=" << calls_after_eager_or
              << " (unexpected if you expected short-circuit)\n\n";

    // This creates a flag for demonstrating a common conditional bug.
    bool flag = true;
    // This shows the correct style: use the flag directly instead of comparing to true.
    bool correct_condition = flag;

    std::cout << "Preferred condition style: if(flag) uses " << correct_condition << " directly\n";

    // This demonstrates a common bug pattern: assignment inside a condition changes the flag.
    // This assigns false to flag and then uses the assigned value as the condition (the condition becomes false).
    bool condition_bug = (flag = false);
    // This shows that flag was modified by mistake (it is now false).
    bool flag_after_bug = flag;

    std::cout << "Assignment-in-condition bug: condition=" << condition_bug << " flag now=" << flag_after_bug
              << " (unexpected if you meant '==')\n";

    // This restores flag for the next example.
    flag = true;
    // This demonstrates the intended comparison: == compares without changing the value.
    bool intended_compare = (flag == false);
    // This shows flag was not changed by the comparison.
    bool flag_after_compare = flag;

    std::cout << "Comparison (==) does not modify: condition=" << intended_compare << " flag now=" << flag_after_compare << "\n\n";

    // This toggles a boolean value using logical NOT (common, readable practice).
    flag = !flag;

    std::cout << "Toggling: after flag = !flag, flag=" << flag << "\n\n";

#if SHOW_UNDEFINED_BEHAVIOR
    // This declares a bool without initializing it (its value is indeterminate).
    bool ub;
    // This reads the uninitialized bool; this is undefined behavior (the program may do anything).
    if (ub)
    {
        std::cout << "UNDEFINED BEHAVIOR path taken (this output is not reliable)\n";
    }
#endif

#if SHOW_COMPILE_ERRORS
    // This creates a void expression.
    void bad_void_expression = void();
    // This attempts to convert void to bool, which is not allowed (compile-time error).
    bool bad_from_void = void();
#endif

#if SHOW_PLATFORM_DEPENDENT
    // This note line is intentionally kept as a print-only observation rather than a size/representation lesson.
    std::cout << "Note: For built-in bool, the most important portability concerns are evaluation/side-effects (shown above), not numeric range.\n";
#endif

    // This returns 0 to signal successful completion.
    return 0;
}

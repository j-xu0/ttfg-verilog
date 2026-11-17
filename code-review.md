# independent analysis
Read each independent submodule, understanding the inputs and outputs and identifying all potential sources of control flow from input to output. Identify edge cases that may lead to unexpected behaviour and bugs: trace undefined values, identify cases where outputs will not match the spec. Deal in the concrete not the abstract.

Identify dependencies on remaining standalone reference modules that are not actually included in the top; these are code bloat / legacy maitenance weight and should no longer be preset.

Check functional test coverage: are we testing the functional behaviour of all the standalone code modules: does each have thorough coverage for the various expected behaviour cases, including slightly unexpected behaviour that we still expect to support (eg SPI input that has minor amoutns of noise outside of the hold/setup time, encoder faults, etc), as well as thorough coverage of the real situations we expect to put the module through: asingle e2e run that appears reasonable is not sufficient.

Similarly, identify tests that do not meaningfully exercise behaviour or spend too much effort to test something that could be done more simply.

# comprehensive analysis
Ensure that full end to end behaviour and testing is solid: there should not be top level paths that are untested.

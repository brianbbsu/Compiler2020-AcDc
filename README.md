# AcDc Compiler

## How to test

Test script: `test.sh`

Place testcases under `test` folder.

Suppose you want a new testcase called `tc`:

- There should be a file called `tc(.error|.DCerror)?.ac` (i.e. `tc.ac`, `tc.error.ac` or `tc.DCerror.ac`), which is the input AC code of the testcase. The existence of `.error` in file name indicate this testcase should result in Compile Error. The existence of `.DCerror` in file name indicate this testcase should result in DC runtime error, and NOT compile error.
- There **CAN** be a file called `tc.expected`, containing the expected output of the AC code.

The test script will scan the test folder for all `.ac` file, and for each AC code, it checks that:

- The AC code can be compiled without error (or with error if `.error` flag exists)
- The output DC program can be run without error (or with error if `.DCerror` flag exists)
- (Optionally) Compare the output of the DC program with the expected output.

The test script can be invoked by directly running the script or by running `make test` in `src` directory. 

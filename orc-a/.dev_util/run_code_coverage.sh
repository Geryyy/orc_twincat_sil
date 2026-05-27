#!usr/bin/bash
mkdir -p code_coverage

if [ ! -e ../build/runTests ]; then
    echo "ERROR: Test files have not been built!"
    exit 0
fi

cd ../build
ctest
./runTests
gcovr -r .. --html-details -o ../.dev_util/code_coverage/coverage.html

echo "-------------- Finished code coverage. Saved at .dev_util/code_coverage/coverage.html --------------"
cd ../.dev_util
browse code_coverage/coverage.html

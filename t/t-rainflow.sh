#! /bin/sh

t=t-rainflow

./ex-rainflow > $t.csv
test $? != 0 && exit 99

diff -u rf1-sort.csv $t.csv > $t.diff
test $? = 2 && exit 99

if test -s $t.diff
then
    cat $t.diff
    exit 1
else
    rm -f $t.diff $t.csv
    exit 0
fi


CXX = g++-9

# VM_TEST
VM_TEST=vm_test.out
VM_TEST_OBJ=$(subst .cpp,.o,$(wildcard *.cpp))
CPPFLAGS=-fPIC -std=c++17 -g -Wno-switch -Wextra -W -Wall -Werror -Wl,--no-undefined 

all: vm_test

vm_test: $(VM_TEST)

.PHONY: all clean

$(VM_TEST): $(VM_TEST_OBJ)
	$(CXX) $(CPPFLAGS) -o $@ $^

$(VM_TEST_OBJ): %.o: %.cpp
	$(CXX) -c $(CPPFLAGS) $< -o $@

clean:
	${RM} $(VM_TEST) $(VM_TEST_OBJ)

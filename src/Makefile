CXXFLAGS = -std=c++17 -Wall -O3 -DNDEBUG

all: AcDc

debug: CXXFLAGS := $(filter-out -DNDEBUG,$(CXXFLAGS))
debug: AcDc

AcDc: AcDc.cpp
	$(CXX) $(CXXFLAGS) AcDc.cpp -o AcDc
test: debug
	bash ../test.sh
clean:
	$(RM) AcDc

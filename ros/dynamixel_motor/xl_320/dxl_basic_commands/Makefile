CXX = gcc
CXXFLAGS = -g -O0 -Wall

SEARCH_TARGET = xl320_start
LOOPBACK_TARGET = dxl_loopback

LIB = dxl_hal.o crc.o packet_utils.o communications.o high_level_commands.o

all: $(SEARCH_TARGET) $(LOOPBACK_TARGET)

$(SEARCH_TARGET): dxl_search.o $(LIB)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(LOOPBACK_TARGET): dxl_loopback.o $(LIB)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f *.o && rm -f $(SEARCH_TARGET) $(LOOPBACK_TARGET)

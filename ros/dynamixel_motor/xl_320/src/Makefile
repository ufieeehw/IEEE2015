CXX = g++
CXXFLAGS = -g -O0 -Wno-write-strings

SEARCH_TARGET = dxl_search
LIB = dxl_hal.o crc.o packet_utils.o communications.o high_level_commands.o 

all: $(SEARCH_TARGET) $(LOOPBACK_TARGET)

$(SEARCH_TARGET): dxl_search.o $(LIB)
	$(CXX) $(CXXFLAGS) -o $@ $^ 

clean:
	rm -f *.o && rm -f $(SEARCH_TARGET) 

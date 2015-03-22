CXX = g++
CXXFLAGS =

SEARCH_TARGET = dxl_settings

LIB = dxl_hal.o crc.o packet_utils.o communications.o high_level_commands.o

all: $(SEARCH_TARGET) 

$(SEARCH_TARGET): dxl_settings.o $(LIB)
	$(CXX) $(CXXFLAGS) -o $@ $^


clean:
	rm -f *.o && rm -f $(SEARCH_TARGET)
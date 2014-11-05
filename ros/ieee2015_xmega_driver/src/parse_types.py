#!/usr/bin/python
import os
import sys
import string

class Type_Parser(object):
    '''The intent is to parse the types.h file to create a python data model 
     relating the types there to ROS topics

     Very rudimentary, does not account for weird things in comments, 
     requires rigorous following of whitespace conventions

    '''
    _types_path = os.path.join('..', '..', '..', 'xmega', 'types.h')

    @classmethod
    def read_the_damn_file(self,):
        lines = self.load_types()
        for line in lines:
            line_dict = self.parse_line(line)
            if line_dict is not None:
                print line_dict
        ['kill', 'start', 'keep_alive', 'poll_imu',]

    @classmethod 
    def load_types(self):
        '''load_types() -> list of string lines'''
        # Read the types.h file
        types_file = file(self._types_path)  
        # Extract the string lines from the types file
        types_lines = [line for line in types_file]  
        return types_lines

    @classmethod
    def string_to_dict(self, _string, substring_order):
        substring_list = _string.split()
        line_dict = {}
        for substring, substring_type in zip(substring_list, substring_order):
            line_dict[substring_type] = substring
        return line_dict

    @classmethod
    def parse_line(self, line):
        '''parse_line(text) -> separated
        Example input line:
        #define START_TYPE            0x02 // out: 'start' 
        '''
        # Return failure if #define is not the first 8 characters
        line = line.strip()
        if '#define' not in line[0:8]:
            return None
            
        options = ['mask', 'msg_length', 'out', 'in']
        order_after_slashes = ['property', 'parameter']
        order_before_slashes = ['define_statement', 'type_name', 'hex_name']
        
        if '//' in line:
            content_before_slashes, content_after_slashes = line.split('//')
            line_dict = {}
            line_dict.update(self.string_to_dict(content_before_slashes, order_before_slashes))
            line_dict.update(self.string_to_dict(content_after_slashes, order_after_slashes))
            return line_dict
        else:
            return None


if __name__ == '__main__':
    Type_Parser.read_the_damn_file()
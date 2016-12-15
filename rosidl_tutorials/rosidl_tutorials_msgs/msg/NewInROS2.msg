# New in ROS 2

# bounded strings
string<=42 my_string  # this is a string of maximum length 42 characters


# bounded arrays
bool[<=5] up_to_five_bools_array
uint8[<=10] up_to_ten_uint8s_array
float32[<=2] up_to_two_float32s_array
string[<=20] up_to_twenty_strings_array

# and you can compose them
string<=10[<=2] up_to_two_enty_strings_array # here we define an array of size two at most
# each string of this array can be 10 characters at most

# default values
bool result true
string name "John Doe"


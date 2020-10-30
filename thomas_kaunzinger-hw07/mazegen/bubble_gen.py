#!/usr/bin/env python3

# Imports
import random
import xml.etree.ElementTree as et

# Generates a "bubble" world with lots of bubble loops that a robot would get
# stuck on if attempting to wall-follow

# Constants
k_wall_model = "model://brick_box_3x1x3"
k_bubble_count = 80
k_width = 70
k_height = 70

# Parses the template
# Template 2 is just the original template but with the parsable wall object removed
m_world = et.parse("template2.xml.erb")
m_root = m_world.getroot()

# Main
if __name__ == '__main__':

    # There's only one world tag
    world = m_root[0]

    running_wall_num = 0
    
    # For every bubble you wish to generate...
    for i in range(0, k_bubble_count):
        
        # Finds the position of the bubble
        x_pos = (random.random() * k_width) - (k_width / 2)
        y_pos = (random.random() * k_height) - (k_height / 2)

        # Since walls are 3x1 in dimension, creates 3 walls to make a solid bubble
        for j in range(0, 3):
            
            # Creates a new "include" element
            new_inc = et.SubElement(world, 'include')
            
            # Creates a new "URI" subelement to the include
            new_inc_model = et.SubElement(new_inc, 'uri')
            new_inc_model.text = k_wall_model

            # Creates a new "name" subelement to the include (names incremented sequentially)
            name_str = ''.join(['wall', str(running_wall_num)])
            new_inc_name = et.SubElement(new_inc, 'name')
            new_inc_name.text = name_str

            # Creates a "position" subelement for the running wall
            # position is added to j so we have the three stack adjacent to one another
            pose_str = ''.join([str(x_pos), ' ', str(y_pos + j), ' -2.0 0 0 0'])
            new_inc_pose = et.SubElement(new_inc, 'pose')
            new_inc_pose.text = pose_str

            # Increments the counter for the number of walls
            running_wall_num += 1

    # Writes the XML to worlds/bubble_world.world in the root of hw7
    with open("../worlds/bubble_world.world", "wb") as f:
        m_world.write(f)


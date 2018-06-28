import math
import sys

def parse_command(cmd):
	''' 
		Reads in a single line gcode command and returns dict of subcommands and values 
		Possible subcommands are:
			X, Y, Z : position commands
			E : extrude command
			T : extruder select (0 (left) or 1 (right))
			F : set feedrate
	'''
	cmd_dict = {}
	cmd_split = cmd.split(' ')

	# first word must be G or M code
	if(cmd_split[0][0] == 'G' or cmd_split[0][0] == 'M'):
		cmd_dict['command'] = cmd_split[0]
	else:
		cmd_dict['command'] = 'invalid'
		return cmd_dict

	# iterate over remaining words in command
	for c in cmd_split[1:]:
		cmd_dict[c[0].upper()] = float(c[1:])

	return cmd_dict

def is_travel_cmd(cmd):
	''' Takes dict of single gcode command and returns True if it's an travel command '''
	if(cmd['command'] == 'G1'):
		if('X' in cmd or 'Y' in cmd):
			if(not 'Z' in cmd):
				if(not 'E' in cmd):
					return True # command is of form 'G1 Xx Yy [Ff]'

	return False

def is_extrude_cmd(cmd):
	''' Takes dict of single gcode command and returns True if it's an extrude command '''
	if(cmd['command'] == 'G1'):
		if('X' in cmd or 'Y' in cmd):
			if(not 'Z' in cmd):
				if('E' in cmd):
					return True # command is of form 'G1 Xx Yy Ee [Ff]'

	return False

def cmd_to_file(file_object, cmd_list):
	''' Takes a list of dicts of commands, and writes them to file in gcode format. '''
	# write each command to seperate line
	for cmd in cmd_list:
		if('command' in cmd):
			file_object.write(cmd['command'])
			cmd.pop('command') # so that it won't be written twice
		else:
			file_object.write('G1')
		
		# write the remaining XYZEF commands
		for key, value in cmd.items():
			file_object.write(' ')
			file_object.write(key + '{:.3f}'.format(value))
		file_object.write('\n')

def compensate_extrude(cmd_list, dist, e_dist, feedrate, accel):
	# assumes trapezoidal speed profile, ramping at constant acceleration up to desired feedrate, then
	# decelerating back down to zero by end of move
	# it's possible that each individual move follows the trapezoidal profile - need to check.

	# ignore empty blocks
	if(len(cmd_list) == 0):
		return cmd_list

	accel_frac = 0
	max_speed = feedrate
	# check if desired feedrate is reached based on move distance and acceleration
	# can you assume that the feedrate is the same during an entire block?
	if(feedrate/accel < math.sqrt(dist/accel)): # reaches desired speed
		# distance during acceleration
		accel_frac = (speed**2/accel)/dist
	else:
		accel_frac = 1
		# max speed reached during halfway point of move
		max_speed = math.sqrt(dist*accel)

	# adjust extrusion amounts to tail off at the end of the move
	# assume pump speed is proportional to instantaneous movement speed

	# attempt 1
	# use average move speed, and subtract leaked amount from tail of moves
	C = 0.001 # constant to tune leak adjustment
	avg_speed = (max_speed/2)*accel_frac + max_speed*(1-accel_frac)

	V_leak = C*avg_speed # volume of leaked material

	# check if there's only one command
	if(len(cmd_list) == 1):
		if(V_leak < e_dist):
			cmd_list[0]['E'] -= V_leak
		else:
			cmd_list[0].pop('E')
	else: 
		# iterate through the commands in reverse order (reducing extrusion amount starting from end)
		N = len(cmd_list)
		for i in range(N-1,-1,-1):
			# calculate extrude increment
			if(i==0):
				e_delta = cmd_list[i]['E']
			else:
				e_delta = cmd_list[i]['E'] - cmd_list[i-1]['E']

			if(V_leak > e_delta): # remove entire extrusion amount
				cmd_list[i].pop('E') # remove the extrusion part of the command
				V_leak -= e_delta # leak volume to be corrected is reduced by current extrusion amount
			else:
				# divide command into two new commands, one with extrude and the other without
				extrude_frac = 1 - V_leak/e_delta
				x_prev = cmd_list[i-1]['X']
				y_prev = cmd_list[i-1]['Y']
				x = cmd_list[i]['X']
				y = cmd_list[i]['Y']

				# modify shortened extrude segment
				cmd_list[i]['E'] -= V_leak
				cmd_list[i]['X'] = x_prev + extrude_frac*(x - x_prev)
				cmd_list[i]['Y'] = y_prev + extrude_frac*(y - y_prev)

				# add non-extrude segment
				cmd_list.insert(i+1, {'command':'G1', 'X':x, 'Y':y})

				break

	return cmd_list

def compensate_travel(cmd_list, dist):
	return cmd_list

def main():
	# open a new empty file to write postprocessed gcode
	if(len(sys.argv) == 1):
		print('Error: Include path to gcode file.')
		return
	gcode_file_path = sys.argv[1]
	parsed_file = gcode_file_path.replace('.', '_parsed.')
	f_out = open(parsed_file, 'w')

	in_extrude_block = False
	cmd_list = []
	cartesian_dist = 0
	extrude_dist = 0
	in_travel_block = False
	feedrate = 0
	accel_max = 0
	last_pt = (0,0) # keeps track of the (X,Y) coordinates of the preceding point

	# open original gcode file and read line by line
	with open(gcode_file_path, 'r') as f_in:
		for line in f_in:
			# get dict of sub-commands
			parsed_cmd = parse_command(line)

			if(parsed_cmd['command'] != 'invalid'):
				# update feedrate if it has changed
				if('F' in parsed_cmd):
					feedrate = parsed_cmd['F']

				# write block to file if extrude block has just ended (last command was extrude, but not this one)
				if(in_extrude_block and not is_extrude_cmd(parsed_cmd)):
					# first deal with extrude block, then consider current command
					in_extrude_block = False
					extrude_dist = cmd_list[-1]['E']

					# adjust extrusion commands to compensate for leak at the end of move
					cmd_list = compensate_extrude(cmd_list, cartesian_dist, extrude_dist, feedrate, accel_max)

					# write new extrusion commands to file
					cmd_to_file(f_out, cmd_list)
					cmd_list = []

				# write block to file if travel block has just ended (last command was travel, but not this one)
				if(in_travel_block and not is_travel_cmd(parsed_cmd)):
					in_travel_block = False
					# add segment length to running total
					# but skip first command, since two points are needed for distance calc
					if(len(cmd_list) > 1):
						cartesian_dist += math.sqrt((cmd_list[-1]['X'] - cmd_list[-2]['X'])**2 + (cmd_list[-1]['Y'] - cmd_list[-2]['Y'])**2) # sqrt((x2-x1)^2 + (y2-y1)^2)
					else: # if it's a single command move
						cartesian_dist = math.sqrt((cmd_list[-1]['X'] - last_pt[0])**2 + (cmd_list[-1]['Y'] - last_pt[1])**2)

					# add purge command at end of travel
					cmd_list = compensate_travel(cmd_list, cartesian_dist)

					# write new travel and purge commands to file
					cmd_to_file(f_out, cmd_list)
					cmd_list = []

				# check if it's an extrude command
				if(is_extrude_cmd(parsed_cmd)):
					# reset variables at start of new block
					if(not in_extrude_block):
						in_extrude_block = True
						cartesian_dist = 0
						extrude_dist = 0

					# append dict of command values
					tmp = {'X':parsed_cmd['X'], 'Y':parsed_cmd['Y'], 'E':parsed_cmd['E'], 'F':feedrate}
					cmd_list.append(tmp)

					# add segment length to running total
					# but skip first command, since two points are needed for distance calc
					if(len(cmd_list) > 1):
						cartesian_dist += math.sqrt((cmd_list[-1]['X'] - cmd_list[-2]['X'])**2 + (cmd_list[-1]['Y'] - cmd_list[-2]['Y'])**2) # sqrt((x2-x1)^2 + (y2-y1)^2)
					else: # if it's a single command move
						cartesian_dist = math.sqrt((cmd_list[-1]['X'] - last_pt[0])**2 + (cmd_list[-1]['Y'] - last_pt[1])**2)

				# check if it's a travel command
				if(is_travel_cmd(parsed_cmd)):
					# reset variables at start of new block
					if(not in_travel_block):
						in_travel_block = True
						cartesian_dist = 0

					# append dict of command values
					tmp = {'X':parsed_cmd['X'], 'Y':parsed_cmd['Y'], 'F':feedrate}
					cmd_list.append(tmp)

				# no processing if it isn't a travel or extrude command
				if((not is_travel_cmd(parsed_cmd)) and (not is_extrude_cmd(parsed_cmd))):
					# set acceleration
					if(parsed_cmd['command'] == 'M204'):
						accel_max = parsed_cmd['S']
					cmd_to_file(f_out, [parsed_cmd])
					cmd_list = []

		if(len(cmd_list) != 0):
			cmd_to_file(f_out, cmd_list)


if __name__ == '__main__':
	main()
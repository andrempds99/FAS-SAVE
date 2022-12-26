import math
import sys
import os
import random
from PIL import Image
from PIL import ImageOps
import numpy as np
import libs.ssim as ssim
import libs.utils as ut
import ctls.mpc as mpccontroller
import ctls.random as randomcontroller
import ctls.bangbang as bangbangcontroller
import ctls.pid as pidcontroller
from simple_pid import PID


#Transforms each frame into a matrix values
def image_to_matrix(path):
	img = Image.open(str(path))
	img = ImageOps.grayscale(img)
	img_data = img.getdata()
	img_tab = np.array(img_data)
	w,h = img.size
	img_mat = np.reshape(img_tab, (h,w))
	return img_mat

#Computes ssim to output the Quality parameter
def compute_ssim(path_a, path_b):
	matrix_a = image_to_matrix(path_a)
	matrix_b = image_to_matrix(path_b)
	return ssim.compute_ssim(matrix_a, matrix_b)

#Encoder Function (If we have time we can improve it ???)
def encode(i, frame_in, frame_out, quality, sharpen, noise):
	framename = str(i).zfill(8) + '.jpg'
	img_in = frame_in + '/' + framename
	img_out = frame_out + '/' + framename
	# generating os command for conversion
	# sharpen actuator
	if sharpen != 0:
		sharpenstring = ' -sharpen ' + str(sharpen) + ' '
	else:
		sharpenstring = ' '
	# noise actuator
	if noise != 0:
		noisestring = ' -noise ' + str(noise) + ' '
	else:
		noisestring = ' '
	# command setup
	command = 'convert {file_in} -quality {quality} '.format(
			file_in = img_in, quality = quality)
	command += sharpenstring
	command += noisestring
	command += img_out
	# executing conversion
	os.system(command)
	# computing current values of indices
	current_quality = compute_ssim(img_in, img_out)
	current_size = os.path.getsize(img_out)
	return (current_quality, current_size)

# -------------------------------------------------------------------

def main(args):

	# Parsing Arguments #

	#Arguments parsed by the ./run.sh {controller} {setpoint_ssim} {setpoint_size}
	mode = args[1] # {controller} 
	setpoint_quality = float(args[5]) # {setpoint_ssim} 
	setpoint_compression = float(args[6]) #{setpoint_size}

	#Arguments parsed 
	folder_frame_in = args[2]  #original frames of the video "\save-master\frames\test\orig"
	folder_frame_out = args[3] #processed frames of the video "\save-master\frames\test\proc"
	folder_results = args[4] #results that will be parsed for the csv file

	#############################

	# Opens video and saves every frame for processing
	path, dirs, files = os.walk(folder_frame_in).next()
	frame_count = len(files)
	final_frame = frame_count + 1
	log = open(folder_results + '/results.csv', 'w')

	#Activates the specific controller
	if mode == "mpc":
		controller = mpccontroller.initialize_mpc()
	elif mode == "random":
		controller = randomcontroller.RandomController()
	elif mode == "bangbang":
		controller = bangbangcontroller.BangbangController()
	elif mode == "pid":
		controller = pidcontroller.PIDController((0, 5), (1, 100), setpoint_quality, setpoint_compression)
	
	#Initial values for actuators
	ctl = np.matrix([[100], [0], [0]])
	
	#For every frame round the value of each item IMPORTANT: 0=Quality; 1=Sharpen; 2=Noise;
	for i in range(1, final_frame):
		# main loop
		ut.progress(i, final_frame) # display progress bar
		quality = np.round(ctl.item(0))
		sharpen = np.round(ctl.item(1))
		noise = np.round(ctl.item(2))

		# Call the Encoding fucntion and encodes the respective frame
		(current_quality, current_size) = \
			encode(i, folder_frame_in, folder_frame_out, quality, sharpen, noise)
		log_line = '{i}, {quality}, {sharpen}, {noise}, {ssim}, {size}'.format(
			i = i, quality = quality, sharpen = sharpen, noise = noise,
			ssim = current_quality, size = current_size)
		print >>log, log_line
		#print(log_line)

		#Matrix for the setpoints we try to achieve and our current outputs
		setpoints = np.matrix([[setpoint_quality], [setpoint_compression]])
		current_outputs = np.matrix([[current_quality], [current_size]])
		
		#Call the function that the computes the actuators for the next frame
		if mode == "mpc":
			try:
				ctl = controller.compute_u(current_outputs, setpoints)
			except Exception as ex:
				print(ex)
		elif mode == "random":
			ctl = controller.compute_u()
		elif mode == "bangbang":
			ctl = controller.compute_u(current_outputs, setpoints)
		elif mode == "pid":
			ctl_tup = controller.compute_u(current_quality, current_size)
			ctl = np.matrix([[ctl_tup[0]], [ctl_tup[1]], [ctl_tup[2]]])

if __name__ == "__main__":
	main(sys.argv)


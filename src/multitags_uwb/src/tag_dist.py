#!/usr/bin/env python3
import numpy as np
from numpy.linalg import multi_dot
from numpy import linalg

class tag_dist:
	ID
	dist
	dist_diff
	diff_var1
	diff_var2
	anch_str
	temp_anch_str
	serial_inst
	Data_store
	measurement
	filt_dist
	filt_xhat
	filt_P


	def __init__(self,inp_ID,inp_serial_inst,inp_anch_str):
		self.ID = inp_ID
		self.dist = 0
		self.dist_diff = 0
		self.serial_inst = inp_serial_inst
		self.anch_str = inp_anch_str
		self.temp_anch_str = None
		self.Data_store = {0.0,0.0}
		self.measurement = {0.0,0.0}
		self.filt_dist = 0.0
		self.filt_xhat = np.zeros([3,1])
		self.filt_P = np.eye(3)


	def dist(self):
		output = [0]*3
		n = 0
		data0 = self.serial_inst.readline()
		Addrs = data0[0]
		for i in range(2,len(data0)):
			if data0[i]==32:
				n = i-1
				break
		data = 0
		for j in range(2,n):
			data = data + ( (data0[j] - 48)*(10**(n-j)) )
		data = data + (data0[n] - 48)
		self.measurement[0] = Addrs
		self.measurement[1] = data


	def dist_collect(self):
		self.serial_inst.open()
		self.serial_inst.flush()
		while (self.temp_anch_str==None):
			self.serial_inst.write(self.anch_str.encode())
			self.temp_anch_str = self.serial_inst.readline()
		if (self.temp_anch_str != None):
			self.dist()
			if (self.measurement[1] > 1):
				self.Data_store[0] = self.measurement[0]
				self.Data_store[1] = self.measurement[1]
			if (self.measurement[1] <= 1):
				self.measurement[0] = self.Data_store[0]
				self.measurement[1] = self.Data_store[1]
			self.dist = self.measurement[1]
			self.temp_anch_str = None
		self.serial_inst.close()



	def dist_filter(self):
		A = np.zeros([3,3])
		A[0][1] = 1.0
		A[1][2] = 1.0
		#----------------------------------------
		B = np.zeros([3,1])
		B[2][0] = 1.0
		#----------------------------------------
		C = np.zeros([1,3])
		C[0][0] = 1.0
		#----------------------------------------
		q = 1.0
		r = 1.0
		eta = 0.5
		Q = np.eye(3)*q
		R = r
		R_inv = 1.0/R
		#-------------------------------------------------------------------------
		#-------------------------------------------------------------------------
		u = 0.0
		y = self.dist
		x_hat = self.filt_xhat
		P = self.filt_P
		dt = 0.1 #0.1*dt_inp
		#-------------------------------------------------------------------------
		P_dot_1 = np.dot(A,P)
		P_dot_2 = np.dot(P,np.transpose(A))
		P_dot_3 = np.dot(Q,np.transpose(Q))
		P_dot_4 = multi_dot([P,np.transpose(C),C,P]) * -1.0 * R_inv
		P_dot = np.add(P_dot_1,P_dot_2)
		P_dot = np.add(P_dot,P_dot_3)
		P_dot = np.add(P_dot,P_dot_4)
		#------------------------------------------------------------------------
		x_hat_dot_1 = np.dot(A,x_hat)
		x_hat_dot_2 = np.dot(C,x_hat) * -1.0
		x_hat_dot_3 = np.add(y,x_hat_dot_2)
		x_hat_dot_4 = eta * multi_dot([P,np.transpose(C),x_hat_dot_3]) * R_inv
		x_hat_dot = np.add(x_hat_dot_1,x_hat_dot_4)
		#------------------------------------------------------------------------
		self.filt_P = P + (P_dot*dt)
		self.filt_xhat = x_hat + (x_hat_dot*dt)
		self.filt_dist = x_hat[0][0]

	def dist_differentiate(self,dt_inp):
		k1 = 10.0
		k2 = 1.0
		local_diff = diff_var1 - filt_dist
		dist_diff = ( -k1 * np.sqrt(np.abs(local_diff)) * np.sign(local_diff) ) + diff_var2
		D_variable1 = dist_diff
		D_variable2 = -k2 * np.sign(local_diff)
		diff_var1 = diff_var1 + (D_variable1 * dt_inp)
		diff_var2 = diff_var2 + (D_variable2 * dt_inp)

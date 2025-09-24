#******************************************************************************
# File Name:   client.py
#
# Description: A simple python based TCP client.
#
#******************************************************************************
# Copyright 2019-2024, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#******************************************************************************

import sys
import socket

if len(sys.argv) != 3:
	print("Usage: python " + sys.argv[0] + " <Remote IP Address> <Port Number>")
	sys.exit()


print("================================================================================")
print("TCP Client")
print("================================================================================")

BUFFER_SIZE = 1024

host=sys.argv[1]
port= int(sys.argv[2])

client = socket.socket()
client.connect((host, port))

message = input("-> ")

while message.lower().strip() != 'exit':
	client.send(message.encode())

	data = client.recv(BUFFER_SIZE).decode()

	print("Received from server: " + data)

	message = input("-> ")

client.close()

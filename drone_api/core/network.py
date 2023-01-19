"""Handles the network communication with the drone.

Three cases are expected:
 - The experiment is running on the drone itself (no network)
 - The experiment is running on a computer connected to the drone via WiFi
 - The experiment is running on a computer connected to multiple drones via WiFi
"""
import socket

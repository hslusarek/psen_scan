<!-- 
Copyright (c) 2020 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
-->

# Acceptance Test PSENscan using Rviz

## Prerequisites
  - Properly configure the PSENscan safety laser scanner.
  - Connect it to power and ethernet.

### Test Sequence

  0. Wait for the PSENscan to be fully powered up.

  1. Run
  ```
  roslaunch psen_scan psen_scan.launch sensor_ip:=<sensor_ip> host_ip:=<host_ip> host_udp_port:=<host_udp_port>
  ```
  Replace the variables in brackets with appropriate values for your setup.

  2. Place your hand in front of the PSENscan.

  3. Move your hand left and right around the PSENscan.

### Expected Results

  0. The PSENscan screen shows a coloured ring with text in the middle.

  1. Rviz shows a red laserscan centered around the origin that matches your surroundings as well as some axes.

  2. Rviz shows distance values near to the center toward the X-Axis.

  3. Rviz shows distance values near to the center moving left and right around the origin in synch with your hand movements.

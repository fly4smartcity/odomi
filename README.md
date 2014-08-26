odomi
=====

ODOMI (Open Data Oriented MIssion Planner) navigation stack

ODOMI is a set of ROS packages for *mission management* and *path planning* for UAVs
and possibly other robotic platforms which exploits open data and knowledge base 
available over the Internet.

The code has been developed and tested using
ROS Hydro on Ubuntu 12.04 64bit and Ubuntu 13.04 64bit.


Dependencies:
-------------
[GDAL 1.10.1] (http://trac.osgeo.org/gdal/wiki/DownloadSource)


HowTo:
------

ROS NODES:
* `rosrun open_data_driver open_data_driver`
* `rosrun mission_planner mission_planner`
* `rosrun odomi_path_planner odomi_path_planner`

GUI:
* `rosrun drone drone`
* `rosrun pubstartgoal pubstartgoal`
* [Generate](https://code.google.com/apis/console) your personal Gmaps API key. Then paste it in the `index.html` file:
    `src="https://maps.googleapis.com/maps/api/js?key=XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX">`
* In the `subscriber_ros_gui.js` be sure to set the `url` parameter in the `ros` variable to match your rosbridge configuration and
* `roslaunch rosbridge_server rosbridge_websocket.launch` together with your custom ROS environment;
* Open `index.html` with a recent browser.

![Odomi screenshot](https://lh6.googleusercontent.com/-0DDY-a9rwGk/U9leyTGOGwI/AAAAAAAAEGQ/PLWV1RZ65pg/w753-h432-no/2014-07-30 "Odomi screenshot")


LICENSE:
--------
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

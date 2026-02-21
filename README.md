<h1>All-Seeing-Eye</h1>

This is the All Seeing Eye, a tentacle (not hentai)
<br>If you have any questions or need permissions feel free to reach!
<br>My discord is: crookisland

<h3>CAD</h3>
The CAD files can be found <a href="https://rit.onshape.com/documents/d315db12e19ef341968bee84/w/e68e0609d778217c288e8294/e/0abaf8f6904bf0725bb47e0d">here</a>

Add short description  of CAD

<h3>Shared Drive</h3>
The shared drive is <a href="https://drive.google.com/drive/folders/0AFuXpOnNKiUpUk9PVA?dmr=1&ec=wgc-drive-hero-goto">here</a>
<br>This contains all previous budgets along with a bunch of older files


<h2>Code</h2>

The code is made up of three main files, one for each controller

<h3>Eye Code:</h3>
The eye code reads the accelerometer on the bottom of the eye, receives commands from the pi, and moves the eyeball and eyelids
Commands:
<code><br><br>
moveMtr &lt;mtr select> &lt;position (0,180)>
<br>mtr 0: topEyelid
<br>mtr 1: bottomEyelid
<br>mtr 2: topEye
<br>mtr 3: bottomEye
<br><br>
pointEye &lt;point 1>, &lt;point 2>, &lt;point 3>
<br>select a point in 3D cartesian coordinates and move eye to that pos
<br>
<br>dumpData &lt;cmd> &lt;time>
<br>cmd = t: dumpData = true
<br>cmd = f: dumpData = false
<br>cmd = c: change sendRate to time
<br>time: transfer speed in ms
<br>
<br>sendData
<br>sends data
<br>
<br>Data packet format
<br>&lt;yaw>, &lt;pitch>, &lt;roll>, &lt;error code>
</code>


<h3>Tentacle Code:</h3>
The tentacle code receives commands from the pi and moves the tentacle accordingly


<h3>picode:</h3>
This reads video from the camera, does vision processing, and commands the tentacle and the eye
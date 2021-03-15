// MIT Kevin Walchko (c) 2021

$fn=90;

use <screws.scad>;

thick = 4;
bottom = 5;
lip = 1;
slide = 10;

module cameraHoles(nut=false){
    translate([15.5, 18,0]) {
        cylinder(h=14, d=2.2);
        if(nut){M2Nut(5+2);}
    }
    translate([70.5, 18,0]) {
        cylinder(h=14, d=2.2);
        if(nut){M2Nut(5+2);}
    }
}

module camera(){
    color("gray") difference()
    {
        cube([78, 27, 8]);
        translate([0,0,-2]){
            translate([-2,5,0]) cube([4,17,14]);
            /* translate([15.5, 18,0]) cylinder(h=14, d=4);
            translate([70.5, 18,0]) cylinder(h=14, d=4); */
            cameraHoles();
        }
    }

    color("#4d4d4d"){
        translate([29, 18, 1]) cylinder(h=7.5, d=5);
        translate([59, 18, 1]) cylinder(h=7.5, d=5);
    }
}

module boardHoles(nut=false){
    dia = 2.5;
    xx = 20.31;
    yy = 12.7;
    translate([2.54,2.54,0]) {
        cylinder(h=14,d=dia);
        if(nut){M2Nut(2+2);}
    }
    translate([2.54,2.54+yy,0]) {
        cylinder(h=14,d=dia);
        if(nut){M2Nut(2+2);}
    }
    translate([2.54+xx,2.54,0]) {
        cylinder(h=14,d=dia);
        if(nut){M2Nut(2+2);}
    }
    translate([2.54+xx,2.54+yy,0]) {
        cylinder(h=14,d=dia);
        if(nut){M2Nut(2+2);}
    }
}

module board(clr="darkslategray"){
    // QWIIC connectors
    color("gray") {
        translate([0,5.6,0]) cube([5,6,5]);
        translate([20.3,5.6,0]) cube([5,6,5]);
    }

    // board
    color(clr) difference()
    {
        cube([25.3,17.7,1.5]);
        translate([0,0,-2]){
            boardHoles();
        }
    }
}

module qtpy(){
    // QWIIC connectors
    color("gray") translate([15.574,17.8/2-3,1.5]) cube([5,6,5-1.5]);

    // USB-C connector
    color("silver") translate([-1, 17.8/2-4.5,1.5]) cube([7.5,9,3]);

    // board
    color("darkslategray") cube([20.574,17.78,1.5]);

}

module irCamera(){
    cube([26,26,1.5]);
    translate([13,13,0]) cylinder(d=8, h=8);
}

module plate(thick){
    h = 65;  // y
    cw = 90; // x
    w = cw;
    difference()
    {
        union(){
            cube([w,h,thick]);
            // camera mount
            mnty = 10;
            mntr = 8;
            translate([0,30+19.5-mnty/2,0]) cube([80+3,mnty,thick+8]);
            translate([81,30,0]) cylinder(d=mntr, h=thick+8);
            translate([5,30,0]) cylinder(d=mntr, h=thick+8);
        }
        translate([10,0,-2]){
            // usb-c cable hole
            I change the width from 15mm to 13 mm double check
            translate([-20,6.5,0]) cube([20,13,thick+4]);
            translate([0,12.5,0]) cylinder(h=thick+4, d=13);

            // qtpy divit
            buff = 0.5;
            translate([2,4,thick]) cube([20.574+buff,17.78+buff,1.5+2]);

            // zip tye cutouts
            translate([15,3,0]) {
                cube([3,4,thick+4]);
                translate([0,19,0]) cube([3,4,thick+4]);
            }
        }

        translate([0,0,-2]){
            translate([3,30,0]) cameraHoles(true); //camera
            translate([slide+3,bottom+20,0]) boardHoles(true); // green
            translate([slide+3+35,bottom+20,0]) boardHoles(true); // blue
            translate([slide+3+35,bottom,0]) boardHoles(true);     // red
        }
    }

    // wall
    /* z = 21-thick;
    translate([0,0,thick]) cube([w,2,z]);
    translate([0,h-2,thick]) cube([w,2,z]);
    translate([w-2,0,thick]) cube([2,h,z]); */

}

// main base plate everything is mounted on
plate(thick);

/* translate([3,30,thick+8]) camera();

translate([slide+3,bottom,thick-1.5]) qtpy();

translate([slide+3,bottom+20,thick+lip]) board("green");   // pressure
translate([slide+3+35,bottom+20,thick+lip]) board("blue"); // imu
translate([slide+3+35,bottom,thick+lip]) board("red");     // light

translate([40,4,10]) irCamera(); */
/* translate([3+30,3,thick]) board();
translate([3+35*2,3,thick]) board();
translate([3+35*3,3,thick]) board(); */

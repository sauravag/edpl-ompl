clc;clear all;close all;

% every raw number is in inches. They get converted to meters at the end (when we draw
% them).

sanity_checks = 1;

L_size = 11;

origin = [0;0];
v_out(:,1) = origin;
L = [];

%% Landmarks in forward direction
x = v_out(1,end);
y = v_out(2,end);

% id = 1;
% origin_to_L1 = 89.5;
% L(:,end+1) = [id; x+origin_to_L1; y];

id = 75;
origin_to_L75 = 100.5;
L(:,end+1) = [id; x+origin_to_L75; y; pi/2];

id = 135;
origin_to_L135 = origin_to_L75 + L_size;
L(:,end+1) = [id; x+origin_to_L135; y; pi/2];

id = 136;
origin_to_L136 = origin_to_L135 + L_size;
L(:,end+1) = [id; x+origin_to_L136; y; pi/2];

id = 2;
origin_to_L2 = 186;
L(:,end+1) = [id; x+origin_to_L2; y; pi/2];


id = 137;
origin_to_L137 = origin_to_L2 + L_size;
L(:,end+1) = [id; x+origin_to_L137; y; pi/2];

id = 138;
origin_to_L138 = origin_to_L137 + L_size;
L(:,end+1) = [id; x+origin_to_L138; y; pi/2];

id = 3;
origin_to_L3 = 282;
L(:,end+1) = [id; x+origin_to_L3; y; pi/2];

id = 4;
origin_to_L4 = 300 + 78;
L(:,end+1) = [id; x+origin_to_L4; y; pi/2];

id = 92;
origin_to_L92 = origin_to_L4 + L_size;
L(:,end+1) = [id; x+origin_to_L92; y; pi/2];

id = 93;
origin_to_L93 = 300 + 166;
L(:,end+1) = [id; x+origin_to_L93; y; pi/2];

id = 94;
origin_to_L94 = origin_to_L93 + L_size;
L(:,end+1) = [id; x+origin_to_L94; y; pi/2];

id = 95;
origin_to_L95 = origin_to_L94 + L_size;
L(:,end+1) = [id; x+origin_to_L95; y; pi/2];

id = 96;
origin_to_L96 = 300 + 259.5;
L(:,end+1) = [id; x+origin_to_L96; y; pi/2];

id = 6;
origin_to_L6 = origin_to_L96 + L_size;
L(:,end+1) = [id; x+origin_to_L6; y; pi/2];

id = 97;
origin_to_L97 = origin_to_L6 + L_size;
L(:,end+1) = [id; x+origin_to_L97; y; pi/2];

id = 98;
origin_to_L98 = 600 + 12;
L(:,end+1) = [id; x+origin_to_L98; y; pi/2];

id = 211;
origin_to_L211 = origin_to_L98 + L_size;
L(:,end+1) = [id; x+origin_to_L211; y; pi/2];

id = 99;
origin_to_L99 = origin_to_L211 + L_size;
L(:,end+1) = [id; x+origin_to_L99; y; pi/2];

%% Vertices
origin_to_digital_door_right = 641.25;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x+origin_to_digital_door_right  ; y];

ToRightMetalCorner = 9.5;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y+ToRightMetalCorner];

ToDigitalDoorCorner = 3.35;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x+ToDigitalDoorCorner ; y];

digital_door_size = 71;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y+digital_door_size];

%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 81;
LeftDoorCornerTo81 = L_size/2;
L(:,end+1) = [id; x; y - LeftDoorCornerTo81; pi];

id = 82;
LeftDoorCornerTo82 = LeftDoorCornerTo81 + L_size;
L(:,end+1) = [id; x; y - LeftDoorCornerTo82; pi];

id = 83;
LeftDoorCornerTo83 = LeftDoorCornerTo82 + L_size;
L(:,end+1) = [id; x; y - LeftDoorCornerTo83; pi];

%% vertices

ToLeftMetalCorner = 3.35;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-ToLeftMetalCorner ; y];

ToHallwayCorner = 9.5;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y+ToHallwayCorner ];

ToRightMetalofBackDoor = 8.25;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-ToRightMetalofBackDoor ; y ];

ToRightCornerofBackDoor = 4;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y+ToRightCornerofBackDoor  ];

% BackDoorWidth = 35.75;
% BackDoorAngle = atan2(26,25);
% x = v_out(1,end);
% y = v_out(2,end);
% v_out(:,end+1) = [x-BackDoorWidth*cos(BackDoorAngle) ; y+BackDoorWidth*sin(BackDoorAngle) ];

% For sanity check: End of door when it is closed!!
BackDoorWidth = 36;
x = v_out(1,end);
y = v_out(2,end);
EndofTheClosedDoor = [x - BackDoorWidth , y];

BackDoorWidth = 36;
BackDoorAngle = atan2(31,18.5);
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-BackDoorWidth*cos(BackDoorAngle) ; y+BackDoorWidth*sin(BackDoorAngle) ];

%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

BackDoorLandmarkAngles = -pi/2-BackDoorAngle;

id = 84;
BackDoorEdgeTo84 = L_size/2;
L(:,end+1) = [id; x+BackDoorEdgeTo84*cos(BackDoorAngle) ; y-BackDoorEdgeTo84*sin(BackDoorAngle); BackDoorLandmarkAngles ];

id = 12;
BackDoorEdgeTo12 = BackDoorEdgeTo84 + L_size;
L(:,end+1) = [id; x+BackDoorEdgeTo12*cos(BackDoorAngle) ; y-BackDoorEdgeTo12*sin(BackDoorAngle); BackDoorLandmarkAngles ];

id = 85;
BackDoorEdgeTo85 = BackDoorEdgeTo12 + L_size;
L(:,end+1) = [id; x+BackDoorEdgeTo85*cos(BackDoorAngle) ; y-BackDoorEdgeTo85*sin(BackDoorAngle); BackDoorLandmarkAngles ];

%% vertices
BackDoorDepth = 1.75;
perpAngle = BackDoorAngle+pi/2;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-BackDoorDepth*cos(perpAngle) ; y+BackDoorDepth*sin(perpAngle) ];

BackDoorWidth = 35.75;
BackDoorAngle = atan2(31,18.5);
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x+BackDoorWidth*cos(BackDoorAngle) ; y-BackDoorWidth*sin(BackDoorAngle) ];

BackDoorToCrossOnShelf = 32.7;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y+BackDoorToCrossOnShelf ];

CrossOnShelfToShelfCorner = 5.2;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-CrossOnShelfToShelfCorner ; y ];


%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 129;
ShelfCornerTo129 = 42;
L(:,end+1) = [id; x ; y+ShelfCornerTo129 ; pi];

id = 128;
ShelfCornerTo128 = ShelfCornerTo129 + L_size;
L(:,end+1) = [id; x; y + ShelfCornerTo128; pi];

id = 127;
ShelfCornerTo127 = ShelfCornerTo128 + L_size;
L(:,end+1) = [id; x; y + ShelfCornerTo127; pi];

%% vertices
ShelfCornerToCross = 131.5;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x  ; y+ShelfCornerToCross  ];


%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 89;
CrossTo89 = 29.5;
L(:,end+1) = [id; x - CrossTo89 ; y ; -pi/2];

id = 88;
CrossTo88 = CrossTo89 + L_size;
L(:,end+1) = [id; x - CrossTo88; y ; -pi/2];

id = 87;
CrossTo87 = CrossTo88 + L_size;
L(:,end+1) = [id; x - CrossTo87; y ; -pi/2];

id = 71;
CrossTo71 = 108;
L(:,end+1) = [id; x - CrossTo71 ; y ; -pi/2];

id = 70;
CrossTo70 = CrossTo71 + L_size;
L(:,end+1) = [id; x - CrossTo70; y ; -pi/2];

id = 69;
CrossTo69 = CrossTo70 + L_size;
L(:,end+1) = [id; x - CrossTo69; y ; -pi/2];


%% vertices
CrossToEdgeofGoalDesk = 152.67;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x - CrossToEdgeofGoalDesk  ; y];

EdgeofGoalDeskToHypothCross = 4;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x  ; y - EdgeofGoalDeskToHypothCross];

%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 300;
HypothCrossTo300 = 76;
L(:,end+1) = [id; x - HypothCrossTo300; y; -pi/2];
 
%% vertices
HypothCrossToEndOfOurDesk = HypothCrossTo300 + L_size/2 + 2;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x - HypothCrossToEndOfOurDesk  ; y];

EdgeofOurDeskToSecondHypothCross = 6;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x  ; y - EdgeofOurDeskToSecondHypothCross];

SecondHypothCrossToMetalCornerInnerDoor = 221.5 - HypothCrossToEndOfOurDesk;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x - SecondHypothCrossToMetalCornerInnerDoor ; y ];

%% Landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 131;
MetalTo131 = 11.4;
L(:,end+1) = [id; x - MetalTo131; y ; -pi/2];

%% vertices
InnerDoorMetalToFrontDoor = 169.5;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x - InnerDoorMetalToFrontDoor ; y ];

ObtainedRestRoomCorridorWidth = v_out(1,end);

%% landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 54;
FrontdoorMetalTo54 = 3.7 + L_size/2;
L(:,end+1) = [id; x ; y + FrontdoorMetalTo54; pi];

id = 24;
FrontdoorMetalTo24 = FrontdoorMetalTo54 + L_size;
L(:,end+1) = [id; x ; y + FrontdoorMetalTo24 ; pi];

id = 53;
FrontdoorMetalTo53 = FrontdoorMetalTo54 + 2*L_size;
L(:,end+1) = [id; x ; y + FrontdoorMetalTo53; pi];

%% vertices
ToSomeWhereAfterRestRoom = 150;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x  ; y + ToSomeWhereAfterRestRoom ];

ToSomeWhereRightofElev = 800;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x + ToSomeWhereRightofElev ; y  ];

ToSomeWhereBehindDigital = 500;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y-ToSomeWhereBehindDigital ];

Toinsidewall = 100;
ToSomeWhereBehindOrigin = ToSomeWhereRightofElev+ObtainedRestRoomCorridorWidth+Toinsidewall;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x-ToSomeWhereBehindOrigin ; y];

ToSomeWhereBehindDavidsOffice = ToSomeWhereBehindDigital;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x; y+ToSomeWhereBehindDavidsOffice ];

ToSomepointOnDavidsWall = Toinsidewall;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x+ToSomepointOnDavidsWall ; y];

% We measured directly the coordinates of the left inner corner of the
% second column (in front of restrooms)
v_out(:,end+1) = [2 ; 273+112+1.2 ];

left_inner_corner_second_column_TO_right_inner_corner_first_column = 267.9;

%% landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 16;
leftInnerCornerSecondColumn_to_L16 = 69;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L16; 0];

id = 15;
leftInnerCornerSecondColumn_to_L15 = 165.1;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L15; 0];

id = 145;
leftInnerCornerSecondColumn_to_L145 = leftInnerCornerSecondColumn_to_L15 + L_size;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L145; 0];

id = 144;
leftInnerCornerSecondColumn_to_L144 = leftInnerCornerSecondColumn_to_L145 + L_size;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L144; 0];

id = 141;
leftInnerCornerSecondColumn_to_L141 = left_inner_corner_second_column_TO_right_inner_corner_first_column - 37;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L141; 0];

id = 142;
leftInnerCornerSecondColumn_to_L142 = leftInnerCornerSecondColumn_to_L141 + L_size;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L142; 0];

id = 14;
leftInnerCornerSecondColumn_to_L14 = leftInnerCornerSecondColumn_to_L142 + L_size;
L(:,end+1) = [id; x; y - leftInnerCornerSecondColumn_to_L14; 0];

%% vertices
left_inner_corner_second_column_TO_right_inner_corner_first_column = left_inner_corner_second_column_TO_right_inner_corner_first_column; % for the sake of code readability
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y - left_inner_corner_second_column_TO_right_inner_corner_first_column];

%% vertices
rightWidthofTheFirstColumn = 14.2;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x + rightWidthofTheFirstColumn  ; y ];

FaceWidthofTheFirstColumn = 25.8;
%% landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 143;
leftOuterCornerFirstColumn_to_L143 = L_size/2;
L(:,end+1) = [id; x; y - leftOuterCornerFirstColumn_to_L143; 0];

id = 140;
leftOuterCornerFirstColumn_to_L140 = FaceWidthofTheFirstColumn  - L_size/2;
L(:,end+1) = [id; x; y - leftOuterCornerFirstColumn_to_L140; 0];

%% vertices
FaceWidthofTheFirstColumn = FaceWidthofTheFirstColumn; % for the sake of code readability
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y - FaceWidthofTheFirstColumn ];

%% vertices
leftWidthofTheFirstColumn = 16.2;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x - leftWidthofTheFirstColumn ; y];

%% landmarks
x = v_out(1,end);
y = v_out(2,end);

id = 13;
leftInnerCornerFirstColumn_to_L13 = 47.5;
L(:,end+1) = [id; x; y - leftInnerCornerFirstColumn_to_L13; 0];

id = 76;
leftInnerCornerFirstColumn_to_L76 = leftInnerCornerFirstColumn_to_L13 + L_size/2;
L(:,end+1) = [id; x; y - leftInnerCornerFirstColumn_to_L76; 0];

%% vertices
left_inner_corner_of_the_first_column_TO_origin = 92.5;
x = v_out(1,end);
y = v_out(2,end);
v_out(:,end+1) = [x ; y - left_inner_corner_of_the_first_column_TO_origin];

% %% Landmarks in backward direction
% x = v_out(1,end);
% y = v_out(2,end);
% 
% id = 76;
% origin_to_L76 = 34.5;
% L(:,end+1) = [id; x; y + origin_to_L76];
% 
% id = 13;
% origin_to_L13 = origin_to_L76 + L_size;
% L(:,end+1) = [id; x; y + origin_to_L13];

% sanity check
if any(v_out(:,end) ~= v_out(:,1))
%     error('origin is not reached')
end

fill(v_out(1,:),v_out(2,:),'b')

%======================================================================================================
%======================================================================================================
%======================================================================================================
%==================================Inside Polygon
MeasuredRestRoomCorridorWidth = 85.5;
MeasuredElhamCorridorWidth = 89.7;
FirstCornerCloseToOrigin = [MeasuredRestRoomCorridorWidth  , MeasuredElhamCorridorWidth];
v_AliOffice(:,1) = FirstCornerCloseToOrigin;

CornerToBackDoorMetal = 511.5;

%% Landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 139;
CornerTo139 = L_size/2;
L(:,end+1) = [id; x + CornerTo139 ; y ; -pi/2];

id = 150;
CornerTo150 = L_size + L_size/2;
L(:,end+1) = [id; x + CornerTo150 ; y ; -pi/2];

id = 59;
CornerTo59 = 32.6 + L_size/2;
L(:,end+1) = [id; x + CornerTo59 ; y ; -pi/2];

id = 7;
CornerTo7 = CornerTo59 + L_size;
L(:,end+1) = [id; x + CornerTo7 ; y ; -pi/2];

id = 60;
CornerTo60 = CornerTo7 + L_size;
L(:,end+1) = [id; x + CornerTo60 ; y ; -pi/2];

id = 61;
CornerTo61 = CornerTo60 + L_size/2 + 62.9 + L_size/2;
L(:,end+1) = [id; x + CornerTo61 ; y ; -pi/2];

id = 8;
CornerTo8 = CornerTo61 + L_size;
L(:,end+1) = [id; x + CornerTo8 ; y ; -pi/2];

id = 62;
CornerTo62 = CornerTo8 + L_size;
L(:,end+1) = [id; x + CornerTo62 ; y ; -pi/2];

id = 63;
CornerTo63 = CornerTo62 + L_size/2 + 63.1 + L_size/2;
L(:,end+1) = [id; x + CornerTo63 ; y ; -pi/2];

id = 9;
CornerTo9 = CornerTo63 + L_size;
L(:,end+1) = [id; x + CornerTo9 ; y ; -pi/2];

id = 64;
CornerTo64 = CornerTo9 + L_size;
L(:,end+1) = [id; x + CornerTo64 ; y ; -pi/2];


id = 65;
CornerTo65 = CornerTo64 + L_size/2 + 62.6 + L_size/2;
L(:,end+1) = [id; x + CornerTo65 ; y ; -pi/2];

id = 10;
CornerTo10 = CornerTo65 + L_size;
L(:,end+1) = [id; x + CornerTo10 ; y ; -pi/2];

id = 66;
CornerTo66 = CornerTo10 + L_size;
L(:,end+1) = [id; x + CornerTo66 ; y ; -pi/2];

id = 67;
CornerTo67 = CornerToBackDoorMetal - 62.5 - 2*L_size - L_size/2;
L(:,end+1) = [id; x + CornerTo67 ; y ; -pi/2];

id = 11;
CornerTo11 = CornerTo67 + L_size;
L(:,end+1) = [id; x + CornerTo11 ; y ; -pi/2];

id = 68;
CornerTo68 = CornerTo11 + L_size;
L(:,end+1) = [id; x + CornerTo68 ; y ; -pi/2];

id = 80;
CornerTo80 = CornerToBackDoorMetal - 18 - L_size/2;
L(:,end+1) = [id; x + CornerTo80 ; y ; -pi/2];

id = 210;
CornerTo210 = CornerToBackDoorMetal - 18 + L_size/2;
L(:,end+1) = [id; x + CornerTo210 ; y ; -pi/2];

%% vertices
CornerToBackDoorMetal = CornerToBackDoorMetal; % for the sake of code readability
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x+CornerToBackDoorMetal  ; y];

% sanity check
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
if x~= EndofTheClosedDoor(1)
    x_fromInnerMeasurments = x
    x_fromouterMeasurments = EndofTheClosedDoor(1)
    differ = x - EndofTheClosedDoor(1)
    error('inconsistency')
end


%% vertices
MetalWidth = 9;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y + MetalWidth];

%% Landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 152;
BackDoorMetalTo152 = L_size/2;
L(:,end+1) = [id; x-BackDoorMetalTo152 ; y ; pi/2];

id = 153;
BackDoorMetalTo153 = L_size+L_size/2;
L(:,end+1) = [id; x-BackDoorMetalTo153 ; y ; pi/2];

%% vertices
ToBoxLine = 24;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - ToBoxLine; y];

%% Landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 72;
BoxLineWallCornerTo72 = 34 + L_size/2;
L(:,end+1) = [id; x ; y+BoxLineWallCornerTo72 ; 0];

id = 73;
BoxLineWallCornerTo73 = BoxLineWallCornerTo72 + L_size;
L(:,end+1) = [id; x ; y+BoxLineWallCornerTo73 ; 0];

id = 74;
BoxLineWallCornerTo74 = BoxLineWallCornerTo73 + L_size;
L(:,end+1) = [id; x ; y+BoxLineWallCornerTo74 ; 0];


%% vertices

BoxLineLength = 96;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y + BoxLineLength];

%% Landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 90;
CrossTo90 = 26 + L_size/2;
L(:,end+1) = [id; x - CrossTo90 ; y ; pi/2];

id = 86;
CrossTo86 = CrossTo90 + L_size;
L(:,end+1) = [id; x - CrossTo86 ; y ; pi/2];

id = 91;
CrossTo91 = CrossTo86 + L_size;
L(:,end+1) = [id; x - CrossTo91 ; y ; pi/2];

%% vertices
ToArnoudDeskCorner = 75;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - ToArnoudDeskCorner ; y ];

ToPoePrinterDesk = 123.9;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - ToPoePrinterDesk ; y ];

EdgeofPoePrinterDeskToCornerofPoePrinterDesk = 18;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y + EdgeofPoePrinterDeskToCornerofPoePrinterDesk ];

%% landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 149;
CrossTo149 = 18.2 + L_size/2;
L(:,end+1) = [id; x - CrossTo149 ; y ; pi/2];

id = 130;
CrossTo130 = 32.6 + L_size/2;
L(:,end+1) = [id; x - CrossTo130 ; y ; pi/2];

id = 148;
CrossTo148 = 45.2 + L_size/2;
L(:,end+1) = [id; x - CrossTo148 ; y ; pi/2];


%% vertices
WidthOfPoePrinterDesk = 60;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - WidthOfPoePrinterDesk; y ];

PoePrinterDeskRightCornerToWall = 62.7;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - PoePrinterDeskRightCornerToWall; y ];

WallToOpenMiddleDoor = 3.6; %6;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y + WallToOpenMiddleDoor];

MiddleDoorLolaToMiddleDoorEnd = 42.4;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - MiddleDoorLolaToMiddleDoorEnd ; y ];

%% landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 147;
EdgeOf_innerDoorTo147 = L_size/2;
L(:,end+1) = [id; x + EdgeOf_innerDoorTo147 ; y ; pi/2];

%% vertices
DoorEndToWall = 9;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y - DoorEndToWall ];

%% landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 134;
EndOfMiddleDoorTo134 = 7.05 + L_size/2;
L(:,end+1) = [id; x - EndOfMiddleDoorTo134 ; y ; pi/2];

id = 132;
EndOfMiddleDoorTo132 = EndOfMiddleDoorTo134 + L_size;
L(:,end+1) = [id; x - EndOfMiddleDoorTo132 ; y ; pi/2];

id = 133;
EndOfMiddleDoorTo133 = EndOfMiddleDoorTo132 + L_size;
L(:,end+1) = [id; x - EndOfMiddleDoorTo133 ; y ; pi/2];

%% vertices
MiddleDoorEndToFontDoorEndAlongTheWall = 81;
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - MiddleDoorEndToFontDoorEndAlongTheWall ; y ];

x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x ; y + DoorEndToWall ];

%% landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 146;
EdgeofFrontDoorTo146 = L_size/2;
L(:,end+1) = [id; x - EdgeofFrontDoorTo146 ; y ; pi/2];

%% vertices
FontDoorEndToRightMetalCornerFrontDoor = 42.5;  %% This is really accurate
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);
v_AliOffice(:,end+1) = [x - FontDoorEndToRightMetalCornerFrontDoor ; y ];

% sanity check
if sanity_checks == 1
LandmarkLineOnFrontDoorToStartVertex = 126.6;
if (abs(v_AliOffice(1,1) - v_AliOffice(1,end))>0.001||abs(v_AliOffice(2,1) + LandmarkLineOnFrontDoorToStartVertex - v_AliOffice(2,end))>0.001 )
    error('inner polygon is not closed!')
end
end

%% Landmarks
x = v_AliOffice(1,end);
y = v_AliOffice(2,end);

id = 151;
FrontDoorTo151 = L_size/2;
L(:,end+1) = [id; x; y - FrontDoorTo151 ; pi];

id = 55;
FrontDoorTo55 = 23 + L_size/2;
L(:,end+1) = [id; x; y - FrontDoorTo55 ; pi];

id = 23;
FrontDoorTo23 = FrontDoorTo55 + L_size;
L(:,end+1) = [id; x; y - FrontDoorTo23 ; pi];

id = 56;
FrontDoorTo56 = FrontDoorTo55 + 2*L_size;
L(:,end+1) = [id; x; y - FrontDoorTo56 ; pi];

id = 57;
FrontDoorTo57 = 83.6 + L_size/2;
L(:,end+1) = [id; x; y - FrontDoorTo57 ; pi];

id = 58;
FrontDoorTo58 = FrontDoorTo57 + L_size;
L(:,end+1) = [id; x; y - FrontDoorTo58 ; pi];

id = 22;
FrontDoorTo22 = FrontDoorTo57 + 2*L_size;
L(:,end+1) = [id; x; y - FrontDoorTo22 ; pi];

%% vertices
v_AliOffice(:,end+1) = v_AliOffice(:,1);

%% ========================= Draw environment in inches
hold on
fill(v_AliOffice(1,:),v_AliOffice(2,:),'b')

title('Environment in Inches')
set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.

%% Draw landmarks
plot(L(2,:), L(3,:), 'r*')
s = [];
L_meters = [L(1,:);L(2:3,:)*0.0254;L(4,:)/pi];
for i = 1:size(L,2)
    text(L(2,i), L(3,i), num2str(L(1,i)), 'fontsize', 12)
    s = [ s , sprintf(['<landmark id = ', num2str(L(1,i)), ' x = ', num2str(L_meters (2,i)), ' y = ', num2str(L_meters (3,i)), ' theta = ', num2str(L_meters (4,i)) , ' /> \n']) ];
end

%% ========================= Draw environment in Meters
figure
hold on
fill(v_out(1,:)*0.0254,v_out(2,:)*0.0254,'b')
fill(v_AliOffice(1,:)*0.0254, v_AliOffice(2,:)*0.0254, 'b')

set(gca,'DataAspectRatio',[1 1 1]); % makes the scaling of different axes the same. So, a circle is shown as a circle not ellipse.
title('Environment in meters')

%% Draw landmarks
plot(L(2,:)*0.0254, L(3,:)*0.0254, 'r*')
% for i = 1:size(L,2)
%     text(L(2,i)*0.0254, L(3,i)*0.0254, num2str(L(1,i)), 'fontsize', 12)
% end

%% Triangulation
Tri_out = DelaunayTri(v_out(1,:)',v_out(2,:)');

% nump = numel(Tri_out.X);
% C = [(1:(nump-1))' (2 :nump)'; nump 1];
% dt = DelaunayTri(uslon, uslat, C);

io = Tri_out.inOutStatus();
figure
patch('faces',Tri_out(io,:), 'vertices', Tri_out.X, 'FaceColor','r');


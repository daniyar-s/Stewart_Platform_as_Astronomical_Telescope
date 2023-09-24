clear all
clc  
r = 350;    % Radius of Secondary mirror
R = 663.32; % Radius of Bearing
D = 530;    % Radius of Telescope
h = 0;
isLonger=0; 
Radius_low = 58.7;                   % Radius of the hexagon
% z_angle    = -40;
z_angle    = 0;
isIntercestion = 1;
while isIntercestion~=0
rotation   = rotz(z_angle);
isIntercestion = 0;
%% Primary mirror(1/2)
%Center of upper circular platform
x = 0;
y = -826.762;
z = 1587.67; 

center_Prim=[x y z];
center_Prim_unRot = center_Prim;
center_Prim = rotation * center_Prim';
center_Prim = center_Prim';
plot3(center_Prim(1), center_Prim(2), center_Prim(3), 'k.', 'MarkerSize', 15);
% center_Prim=transpose(center_Prim);

%% Coordinates of an Object in the sky
% obj_point = [-170 -80 160];
% obj_point = [67.611654728 31.47216762 123.626933041];
% obj_point = [-101.799135321 -111.30749817 123.626933041];
% obj_point = [-3000 3000 8000];
% obj_point = [00  00 12000];
% obj_point = [-1470.799135321 1210.30749817 2871.626933041];
obj_point = [-2080 -300 4678];

% direction_point = [1.2 0 -2.5];
plot3(obj_point(1), obj_point(2), obj_point(3), 'k.', 'MarkerSize', 15);
dir_from_point = -obj_point+center_Prim;

%% Secondary mirror (circle,which is inclined at angle and  located at certain height)
h1=2065; %Height of secondary mirror
%Secondary mirror coordinates
a=[[r 0 h1] ;[r/2  sqrt(3)*r/2  h1] ;[-r/2  sqrt(3)*r/2  h1]; [-r  0 h1] ;[-r/2 -sqrt(3)*r/2 h1] ;[r/2 -sqrt(3)*r/2 h1]];
a = (rotation *a')';
center_Sec = (a(4,:)+a(1,:))/2;
center_Sec = (rotation *center_Sec')';
normal_Sec = transpose(rotx(-30)*transpose(cross(a(5,:)-center_Sec,a(6,:)-center_Sec)/norm(cross(a(5,:)-center_Sec,a(6,:)-center_Sec))));
normal_Sec = (rotation * normal_Sec')';
plot3(center_Sec(1), center_Sec(2), center_Sec(3), 'k.', 'MarkerSize', 15);
plotCircle3D(center_Sec,normal_Sec,r)
hold on
plot3([0 0],[-82 -95],[0 0],'Linewidth',5)
pbaspect([1 1 1])

%% Hexagon 
Number = 5;                         % Number of sides
angle = linspace(0, pi, Number+1);  % Angle Vector                                              
phi = pi;                           % Phase (Rotates Figure)
up_hex_x = [661.85 580.63 315.06 -50.54 -400.09 -622.62];
up_hex_y = [44.1 -320.72 -583.72 -661.39 -529.08 -228.78];
up_hex_z = zeros(1,6);
hex = [up_hex_x; up_hex_y; up_hex_z];
hex = rotation * hex;
%coordinates of location of 6 linear actuators
x0=[hex(:,1) hex(:,6) hex(:,4) hex(:,3) hex(:,2) hex(:,5)]; 

% Upper plate coordinates (hexagon vertices)
b=[[-350.48709167; -105.55595847; h]  [-300.59429578; -208.8691723; h]  [83.82940428; 356.30870432; h] [-30.58886139;364.75688253 ;h] [266.65768739; -250.75274585 ;h] [331.18315718; -155.88771023; h] ];
b = rotation * b;

%% Bearing platform
plot3(hex(1,:), hex(2,:),hex(3,:))
p0=[[R; 0; 0] [R/2; sqrt(3)*R/2; 0] [-R/2; sqrt(3)*R/2; 0] [-R; 0 ;0] [-R/2; -sqrt(3)*R/2; 0] [R/2; -sqrt(3)*R/2; 0]];
p0 = rotation * p0;
center_bearing=(p0(:,4)+p0(:,1))/2; %center of lower circle
% center_bearing = rotation * center_bearing;
% normal and center of low circle(bearing)
normal_bearing = transpose(cross(p0(:,5)-center_bearing,p0(:,6)-center_bearing)/norm(cross(p0(:,5)-center_bearing,p0(:,6)-center_bearing)));
% normal_bearing = (rotation * normal_bearing')';
center_bearing=transpose(center_bearing);
plotCircle3D(center_bearing,normal_bearing,R)
plotCircle3D(center_bearing,normal_bearing,D)

%% [Correct] from Primary mirror to Secondary mirror
correct_dir = -center_Prim+center_Sec;
intersection=line_plane_intersection( correct_dir,center_Prim, normal_Sec, center_Sec);
hold on;
H_from_primary = quiver3(center_Prim(1), center_Prim(2), center_Prim(3), correct_dir(1), correct_dir(2), correct_dir(3), 15);
set(H_from_primary,'AutoScale','on', 'AutoScaleFactor',0.8)
ref_correct = (intersection-center_Prim)-2*dot(normal_Sec,intersection-center_Prim)*normal_Sec;


%% Ideal and Correct normal vector calculation
dir_from_point = dir_from_point/norm(dir_from_point);
correct_dir = correct_dir/norm(correct_dir);
theta = atan2(norm(cross(-dir_from_point,correct_dir)),dot(-dir_from_point,correct_dir));
% theta = rad2deg(theta)

normal_vec = [(dir_from_point(1)-correct_dir(1))/(2*sin(theta/2)),
              (dir_from_point(2)-correct_dir(2))/(2*sin(theta/2)),
              (dir_from_point(3)-correct_dir(3))/(2*sin(theta/2))];
normal_vec = normal_vec';
point_on_normal = -normal_vec*10 +center_Prim;
normal_vec = -normal_vec/norm(normal_vec);

% normal_vec_quiver = quiver3(center_Prim(1), center_Prim(2), center_Prim(3), normal_vec(1), normal_vec(2), normal_vec(3), 15);
% set(normal_vec_quiver,'AutoScale','on', 'AutoScaleFactor',80)
% plot3(point_on_normal(1), point_on_normal(2), point_on_normal(3), 'k.', 'MarkerSize', 15);

n1 = atan((point_on_normal(3)-center_Prim(3))/sqrt((point_on_normal(1)-center_Prim(1))^2 +(point_on_normal(2)-center_Prim(2))^2));
n1 = rad2deg(n1);
n1_zx = atan(abs(point_on_normal(3)-center_Prim(3))/abs(point_on_normal(1)-center_Prim(1)));
n1_zx = rad2deg(n1_zx);
% n1_yx = atan(abs(point_on_normal(1)-center_Prim(1))/abs(point_on_normal(2)-center_Prim(2)));
% n1_yx = rad2deg(n1_yx)
n1_zy = atan(abs(point_on_normal(3)-center_Prim(3))/abs(point_on_normal(2)-center_Prim(2)));
n1_zy = rad2deg(n1_zy);
% n1_zy =  90
for k=1:4
    coeff_zx = 1;
    coeff_zy = 1;
    if k==1 || k==3
        angle_diff_zy = 0; 
        angle_diff_zx = 0;
    end
    if angle_diff_zy>0 && z_angle <0
        angle_diff_zy= -1*angle_diff_zy;
    end
    if angle_diff_zx>0 && z_angle <0
        angle_diff_zx= -1*angle_diff_zx;
    end
%     
%     if angle_diff_zy>0 && z_angle >0
%         angle_diff_zy= 1*angle_diff_zy;
%     end
%     if angle_diff_zx>0 && z_angle >0
%         angle_diff_zx= 1*angle_diff_zx;
%     end
    
    if normal_vec(1)<0 && normal_vec(2) >0
        coeff_zx = 1;
        coeff_zy = 1;
    end
    if normal_vec(1)>0 && normal_vec(2) >0 
        coeff_zx = -1;
        coeff_zy = 1;
    end
    if normal_vec(1)>0 && normal_vec(2) <0 
        coeff_zx = -1;
        coeff_zy = -1;
    end
    
    if normal_vec(1)<0 && normal_vec(2) <0 
        coeff_zx =  1;
        coeff_zy = -1;
    end
    
    %% Primary mirror orientation /  Upper platform(2/2)
    thettax = coeff_zy * (n1_zy-90) + angle_diff_zy; %zy
    thettay = coeff_zx * (n1_zx-90) + angle_diff_zx; %zx
    thettaz = 0;
    
    if k<=2
        T = rotx(thettax)*roty(thettay);    %Rotation matrix
    else
        T = roty(thettay)*rotx(thettax);    %Rotation matrix
    end


    P = [x;y;z];     %Position matrix
    P = rotation * P;
    P = [P,P,P,P,P,P];
    
    B = P+T*b;       %Coordinates of upper platform after rotation
    B = [B(:,3),B(:,4),B(:,2),B(:,5),B(:,6),B(:,1)];

    center_Prim=transpose(center_Prim);
    normal_up=transpose(cross(B(:,5)-center_Prim,B(:,6)-center_Prim)/norm(cross(B(:,5)-center_Prim,B(:,6)-center_Prim)));
    center_Prim=transpose(center_Prim);
    if k~=1
%         normal_quiver = quiver3(center_Prim(1), center_Prim(2), center_Prim(3), normal_up(1), normal_up(2), normal_up(3), 15);
%         set(normal_quiver,'AutoScale','on', 'AutoScaleFactor',80)
    end
   
    point_on_act_norm = normal_up*25+center_Prim;
%     plot3(point_on_act_norm(1), point_on_act_norm(2), point_on_act_norm(3), 'k.', 'MarkerSize', 15);
    n2 = atan((point_on_act_norm(3)-center_Prim(3))/sqrt((point_on_act_norm(1)-center_Prim(1))^2 +(point_on_act_norm(2)-center_Prim(2))^2));
    n2 = rad2deg(n2);
    n2_zx = atan(abs(point_on_act_norm(3)-center_Prim(3))/abs(point_on_act_norm(1)-center_Prim(1)));
    n2_zx = rad2deg(n2_zx);
    n2_zy = atan(abs(point_on_act_norm(3)-center_Prim(3))/abs(point_on_act_norm(2)-center_Prim(2)));
    n2_zy = rad2deg(n2_zy);
    
    angle_diff_zx = n1_zx-n2_zx;
    angle_diff_zy = n1_zy-n2_zy;
    
    if k<=2
        angle_diff_zx_1 = angle_diff_zx;
        angle_diff_zy_1 = angle_diff_zy;
        normal_up_1 = normal_up;
        B1 = B;
    else
        angle_diff_zx_2 = angle_diff_zx;
        angle_diff_zy_2 = angle_diff_zy;
        normal_up_2 = normal_up;
        B2 = B;
    end

end

smth1 = sqrt(angle_diff_zx_1^2 + angle_diff_zy_1^2);
smth2 = sqrt(angle_diff_zx_2^2 + angle_diff_zy_2^2);

if smth1<smth2
    normal_up = normal_up_1;
    B = B1;
%     disp('HERE!1')
else
    normal_up = normal_up_2;
    B = B2;
%     disp('HERE!2')
end

    L = B-x0;        %The  vector of length of legs after rotation
    Mag_L(1) = norm(L(:,1)); %Length of legs of stewart platform
    Mag_L(2) = norm(L(:,2)); 
    Mag_L(3) = norm(L(:,3)); 
    Mag_L(4) = norm(L(:,4)); 
    Mag_L(5) = norm(L(:,5)); 
    Mag_L(6) = norm(L(:,6)); 
    Mag_L = Mag_L-1206;
    Mag_L = Mag_L/1000;
    Mag_L_temp = Mag_L(6);
    Mag_L(6) = Mag_L(5);
    Mag_L(5) = Mag_L(4);
    Mag_L(4) = Mag_L(3);
    Mag_L(3) = Mag_L_temp;
    % Check if the leg lengthes are longer than reality
    isLonger=0;
    for bnm = 1:6
        if Mag_L(bnm)>0.7
            isLonger = 1;
        end
    end
%% Visualisation of the first platform
hold on
plotCircle3D(center_Prim,normal_up,r)
plot3(center_Prim(1), center_Prim(2), center_Prim(3), 'k.', 'MarkerSize', 15);
xlabel('x')
ylabel('y')
zlabel('z')
grid
drawnow limitrate
% Drawing Leg1
% plot3(x0(1,1), x0(2,1), x0(3,1), 'k.', 'MarkerSize', 25);
% plot3(B(1,1), B(2,1), B(3,1), 'k.', 'MarkerSize', 25);
H=animatedline;
draw_legX1=[x0(1,1) B(1,1)];
draw_legY1=[x0(2,1) B(2,1)];
draw_legZ1=[x0(3,1) B(3,1)];
for k = 1:length(draw_legX1)
    addpoints(H,draw_legX1(k),draw_legY1(k),draw_legZ1(k));
    drawnow
end
% Drawing Leg2
M=animatedline;
draw_legX2=[x0(1,2) B(1,2)];
draw_legY2=[x0(2,2) B(2,2)];
draw_legZ2=[x0(3,2) B(3,2)];
for k = 1:length(draw_legX2)
    addpoints(M,draw_legX2(k),draw_legY2(k),draw_legZ2(k));
    drawnow
end
% Drawing Leg3
N=animatedline;
draw_legX3=[x0(1,3) B(1,3)];
draw_legY3=[x0(2,3) B(2,3)];
draw_legZ3=[x0(3,3) B(3,3)];
for k = 1:length(draw_legX3)
    addpoints(N,draw_legX3(k),draw_legY3(k),draw_legZ3(k));
    drawnow
end
% Drawing Leg4
PP=animatedline;
draw_legX4=[x0(1,4) B(1,4)];
draw_legY4=[x0(2,4) B(2,4)];
draw_legZ4=[x0(3,4) B(3,4)];
for k = 1:length(draw_legX4)
    addpoints(PP,draw_legX4(k),draw_legY4(k),draw_legZ4(k));
    drawnow
end
% Drawing Leg5
O=animatedline;
draw_legX5=[x0(1,5) B(1,5)];
draw_legY5=[x0(2,5) B(2,5)];
draw_legZ5=[x0(3,5) B(3,5)];
for k = 1:length(draw_legX5)
    addpoints(O,draw_legX5(k),draw_legY5(k),draw_legZ5(k));
    drawnow
end
% Drawing Leg6
I=animatedline;
draw_legX6=[x0(1,6) B(1,6)];
draw_legY6=[x0(2,6) B(2,6)];
draw_legZ6=[x0(3,6) B(3,6)];
for k = 1:length(draw_legX6)
    addpoints(I,draw_legX6(k),draw_legY6(k),draw_legZ6(k));
    drawnow
end

%% Raytracing
number_source = 20;                     % Number of rays                             
p = linspace(0, 2*pi, number_source+1); 
Radius_source=270 ;                      % Radius of telescope located at the bottom, where bearing is located.
phi = pi/4;                             % Phase (Rotates Figure)
x_source = Radius_source*cos(p + phi)+obj_point(1); % Coordinates of rays traced from telescope
y_source = Radius_source*sin(p + phi)+obj_point(2);
% x_center=sum(x_source)/number_source;
% y_center=sum(y_source)/number_source;
x_center=obj_point(1);
y_center=obj_point(2);
x_source=[x_source x_center];
y_source=[y_source y_center];
Source=[x_source;y_source;zeros(1,length(x_source))];
Source(3,:)=obj_point(3);
% Source = rotation * Source;
for i=1:length(x_source)
    origin = transpose(Source(:,i));
    direction =transpose(rotx(0)*transpose(dir_from_point));
    obj_point = origin;
    dir_from_point = direction;
    
    %% Secondary mirror plane with Incident line
    int_secPlane_IncLine = line_plane_intersection( dir_from_point,obj_point, normal_Sec, center_Sec);
%     plot3(int_secPlane_IncLine(1), int_secPlane_IncLine(2), int_secPlane_IncLine(3), 'k.', 'MarkerSize', 15);
    dir_sec_line= -center_Sec + int_secPlane_IncLine;
    temp = [int_secPlane_IncLine;center_Sec];
    dist_inter_center = pdist(temp,'euclidean');
    d = dist_inter_center - r;

    if d <0 || isLonger==1
        isIntercestion = 1;
        z_angle = z_angle+2;
    end

    
    %% Finding Intersection with upper platform
    intersection=line_plane_intersection( dir_from_point,obj_point, normal_up, center_Prim);
    hold on;
    %Plot ray origin
    plot3(obj_point(1), obj_point(2), obj_point(3), 'k.', 'MarkerSize', 15);
    % direction visualisation with quiver function (arrows appear)
    h_from_point=quiver3(obj_point(1), obj_point(2), obj_point(3), dir_from_point(1), dir_from_point(2), dir_from_point(3), 15);
    %Lenght of arrow
    set(h_from_point,'AutoScale','on', 'AutoScaleFactor',400)
    %calculating normal and directon of ref_correctection from secondary mirror
    ref_correct = (intersection-obj_point)-2*dot(normal_up,intersection-obj_point)*normal_up;

    %% Trace the rays with found intersection point from upper platform
    orig = intersection;
    %Direction of ref_correctected ray is normal of the plane at point need to check!!!
    direct = ref_correct/norm(ref_correct);
    %Finding intersection of ref_correctected ray with secondary   mirror 
    intersect=line_plane_intersection( direct,orig, normal_Sec, center_Sec);
    hold on;
    grid on;
    %Ray origin
        plot3(orig(1), orig(2), orig(3), 'k.', 'MarkerSize', 15);
        % direction of rays
        h_from_primary=quiver3(orig(1), orig(2), orig(3), direct(1), direct(2), direct(3), 15);
        set(h_from_primary,'AutoScale','on', 'AutoScaleFactor',400)
        view(60,30);
    % Trace rays with found third intersection
    ref_correctection=(intersect-orig)-2*dot(normal_Sec,intersect-orig)*normal_Sec;
    
    %% from Secondary mirror to Telescope
    plot3(intersect(1), intersect(2), intersect(3), 'k.', 'MarkerSize', 15);
    dir_from_sec = ref_correctection/norm(ref_correctection);
    h_from_sec = quiver3(intersect(1), intersect(2), intersect(3), dir_from_sec(1), dir_from_sec(2), dir_from_sec(3), 15);
    set(h_from_sec,'AutoScale','on', 'AutoScaleFactor',400)
    intersection=line_plane_intersection( dir_from_sec,intersect, normal_bearing, center_bearing);
    plot3(intersection(1), intersection(2), intersection(3), 'k.', 'MarkerSize', 15);
end
    if isIntercestion==1
        close all
    end
    
%     xlim([-1200 1200])
%     ylim([-1200 1200])
%     zlim([0 4000])  
end
disp('Leg lengths:')
%Mag_L

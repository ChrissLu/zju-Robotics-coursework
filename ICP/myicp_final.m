% Robotics ��homework about icp




clear ; 
close all;

x=zeros(10,1);
y=zeros(10,1);
z=zeros(10,1);

ply_p = pcread('0.ply');   %��Ҫƥ��ĵ���,laser_map

robot_tf{1} = eye(4,4);

for i=1:9

tform = eye(4,4);
str = [num2str(i) , '.ply'];  
ply_pp = pcread(str);   %���е���

%������Ƶ�����ÿһ�д���һ����
p_points  =  ply_p.Location;  %p_numner*3���󣬵�������������
pp_points  =  ply_pp.Location;  %180*3����

%��ƥ��
A = robot_tf{i}(1:3,1:3)* pp_points' + robot_tf{i}(1:3,4);
pp_points  =  A';

step=0;
norm_before=0;

while 1
    %Ѱ�������
    p_points_temp = zeros(180,3);
    for j =1:180
    dis=(p_points-pp_points(j,:)).^2;  %�㲥
    distance =dis(:,1) + dis(:,2) + dis(:,3);
    [M,index] = min(distance);        
    %index = findNearestNeighbors(ply_p,pp_points(j,:),1);
    p_points_temp(j,:) = p_points(index,:); 
    end 
   
    
   tform_step = icpstep(p_points_temp ,pp_points);
   tform = tform * tform_step;
   
   %���������任���Լ����������
   A = tform_step(1:3,1:3)* pp_points' + tform_step(1:3,4);
   pp_points  =  A'; 
   norm_now = norm(pp_points - p_points_temp);
   norm1 = abs(norm_before - norm_now);
   norm_before = norm_now;

if  (norm1 < 0.001)||(step>100)
    break
end



step = step +1;

end

robot_tf{i+1} = robot_tf{i} * tform;    
x(i+1,:) = robot_tf{i+1}(1,4);    
y(i+1,:) = robot_tf{i+1}(2,4);    
z(i+1,:) = robot_tf{i+1}(3,4);    
ply_p = pointCloud(p_points);     
ply_pp = pointCloud(pp_points);
ply_p = pcmerge(ply_p, ply_pp, 0.001);




end
tform
f=figure;
pcshow(ply_p, 'MarkerSize', 20);
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
set(f,'Color','black');
plot3(x,y,z,'LineWidth',3);
hold off;

disp('pose for last frame��');
x_terminal = x(10,1)
y_terminal = y(10,1)
z_terminal = z(10,1);
% delta_alpha = atan2(robot_tf{9}(3,2),robot_tf{9} (3,3)); 
% delta_beta = atan2(-robot_tf{9}(3,1),sqrt(robot_tf{9}(3,2)^2+robot_tf{9}(3,3)^2));
delta_gamma = atan2(robot_tf{9}(2,1),robot_tf{9}(1,1))*180/pi;
theta_terminal = delta_gamma

function tform = icpstep(p_points,pp_points)

    p_center = mean(p_points);  % ��ʼ����p�㼯����
    p_points_minus_center = p_points - p_center;   %ÿһ�д���һ���������
    pp_center = mean(pp_points);  % ��ʼ����pp�㼯����  
    pp_points_minus_center = pp_points - pp_center;   %ÿһ�д���һ���������

    W = zeros(3,3);
   
    %���ڱ��������ת�ù������꣬����ת��˳���෴
    W = W + pp_points_minus_center' * p_points_minus_center;  
    
    
    [U,S,V] = svd(W);
    
    R = V'*U';

    t =  p_center' - R * pp_center'; 


    tform = [ R    t
            0 0 0  1];
   	
end

function allowed_regions
close all; clear all;
bx = 4;
by = 3;

l = 2;
w = 0.5;

% section 1
theta = linspace(0, pi/2, 200);
xeqn1 = @(theta)(zeros(size(theta)));
yeqn1 = @(theta)(zeros(size(theta)));
xeqn2 = @(theta)(bx - l*cos(theta) - w*sin(theta));
yeqn2 = @(theta)(by - l*sin(theta));
make_quad(xeqn1, yeqn1, xeqn2, yeqn2, theta)

% plot3(x,y,theta);
% hold on;
% plot3(x, zeros(size(y)), theta);
% plot3(zeros(size(x)), y, theta);
% plot3(zeros(size(x)), zeros(size(x)), theta);
% 
% th1 = theta(1:end-1);
% th2 = theta(2:end);
% zer = zeros(1,size(theta,2) - 1);
% vertx = [zer;xeqn(th1);xeqn(th2);zer;zer];
% verty = zeros(size(vertx));
% vertz = [th1; th1; th2; th2; th1];
% p1 = patch(vertx, verty, vertz, 'r')
% 
% vertx = [xeqn(th1);xeqn(th1);xeqn(th2);xeqn(th2);xeqn(th1)];
% verty = [zer; yeqn(th1); yeqn(th2); zer; zer];
% vertz = [th1; th1; th2; th2; th1];
% p2 = patch(vertx, verty, vertz, 'b')
% 
% vertx = [xeqn(th1);zer;zer;xeqn(th2);xeqn(th1)];
% verty = [yeqn(th1); yeqn(th1); yeqn(th2); yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p3 = patch(vertx, verty, vertz, 'g')
% 
% vertx = [zer;zer;zer;zer;zer];
% verty = [yeqn(th1); zer; zer; yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p4 = patch(vertx, verty, vertz, 'y')
% p1.FaceAlpha = 0.7;
% p2.FaceAlpha = 0.7;
% p3.FaceAlpha = 0.7;
% p4.FaceAlpha = 0.7;
% p1.LineStyle = 'none';
% p2.LineStyle = 'none';
% p3.LineStyle = 'none';
% p4.LineStyle = 'none';
% % section 2
% theta = linspace(pi/2, pi, 200);
% xeqn = @(theta)(bx - w*cos(theta - pi/2));
% xeqn_start = @(theta)(l*cos(pi - theta));
% yeqn = @(theta)(by - l*sin(theta) - w*sin(theta - pi/2));
% x = xeqn(theta);
% y = yeqn(theta);
% 
% plot3(x,y,theta);
% hold on;
% plot3(x, zeros(size(y)), theta);
% plot3(zeros(size(x)), y, theta);
% plot3(zeros(size(x)), zeros(size(x)), theta);
% 
% th1 = theta(1:end-1);
% th2 = theta(2:end);
% zer = zeros(1,size(theta,2) - 1);
% vertx = [xeqn_start(th1) ;xeqn(th1);xeqn(th2);xeqn_start(th2) ;xeqn_start(th1) ];
% verty = zeros(size(vertx));
% vertz = [th1; th1; th2; th2; th1];
% p1 = patch(vertx, verty, vertz, 'r')
% 
% vertx = [xeqn(th1);xeqn(th1);xeqn(th2);xeqn(th2);xeqn(th1)];
% verty = [zer; yeqn(th1); yeqn(th2); zer; zer];
% vertz = [th1; th1; th2; th2; th1];
% p2 = patch(vertx, verty, vertz, 'b')
% 
% vertx = [xeqn(th1);xeqn_start(th1) ;xeqn_start(th2);xeqn(th2);xeqn(th1)];
% verty = [yeqn(th1); yeqn(th1); yeqn(th2); yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p3 = patch(vertx, verty, vertz, 'g')
% 
% vertx = [xeqn_start(th1);xeqn_start(th1);xeqn_start(th2);xeqn_start(th2);xeqn_start(th1)];
% verty = [yeqn(th1); zer; zer; yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p4 = patch(vertx, verty, vertz, 'y')
% p1.FaceAlpha = 0.7;
% p2.FaceAlpha = 0.7;
% p3.FaceAlpha = 0.7;
% p4.FaceAlpha = 0.7;
% p1.LineStyle = 'none';
% p2.LineStyle = 'none';
% p3.LineStyle = 'none';
% p4.LineStyle = 'none';
% 
% 
% % section 3
% theta = linspace(pi, pi*3/2, 200);
% xeqn = @(theta)(bx - w*cos(theta - pi/2));
% xeqn_start = @(theta)(l*cos(pi - theta));
% yeqn = @(theta)(by - l*sin(theta) - w*sin(theta - pi/2));
% x = xeqn(theta);
% y = yeqn(theta);
% 
% plot3(x,y,theta);
% hold on;
% plot3(x, zeros(size(y)), theta);
% plot3(zeros(size(x)), y, theta);
% plot3(zeros(size(x)), zeros(size(x)), theta);
% 
% th1 = theta(1:end-1);
% th2 = theta(2:end);
% zer = zeros(1,size(theta,2) - 1);
% vertx = [xeqn_start(th1) ;xeqn(th1);xeqn(th2);xeqn_start(th2) ;xeqn_start(th1) ];
% verty = zeros(size(vertx));
% vertz = [th1; th1; th2; th2; th1];
% p1 = patch(vertx, verty, vertz, 'r')
% 
% vertx = [xeqn(th1);xeqn(th1);xeqn(th2);xeqn(th2);xeqn(th1)];
% verty = [zer; yeqn(th1); yeqn(th2); zer; zer];
% vertz = [th1; th1; th2; th2; th1];
% p2 = patch(vertx, verty, vertz, 'b')
% 
% vertx = [xeqn(th1);xeqn_start(th1) ;xeqn_start(th2);xeqn(th2);xeqn(th1)];
% verty = [yeqn(th1); yeqn(th1); yeqn(th2); yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p3 = patch(vertx, verty, vertz, 'g')
% 
% vertx = [xeqn_start(th1);xeqn_start(th1);xeqn_start(th2);xeqn_start(th2);xeqn_start(th1)];
% verty = [yeqn(th1); zer; zer; yeqn(th2); yeqn(th1)];
% vertz = [th1; th1; th2; th2; th1];
% p4 = patch(vertx, verty, vertz, 'y')
% p1.FaceAlpha = 0.7;
% p2.FaceAlpha = 0.7;
% p3.FaceAlpha = 0.7;
% p4.FaceAlpha = 0.7;
% p1.LineStyle = 'none';
% p2.LineStyle = 'none';
% p3.LineStyle = 'none';
% p4.LineStyle = 'none';
daspect([1,1,1]);
grid on;
camlight HEADLIGHT;
xlabel('x');
ylabel('y');
zlabel('\theta');

function make_quad(xeqn1, yeqn1, xeqn2, yeqn2, theta)
hold on;
th1 = theta(1:end-1);
th2 = theta(2:end);

vertx = [xeqn1(th1); xeqn2(th1); xeqn2(th2); xeqn1(th2); xeqn1(th1)];
verty = [yeqn1(th1); yeqn1(th1); yeqn1(th2); yeqn1(th2); yeqn1(th1)];
vertz = [th1; th1; th2; th2; th1];
p1 = patch(vertx, verty, vertz, 'r')

vertx = [xeqn2(th1); xeqn2(th1); xeqn2(th2); xeqn2(th2); xeqn2(th1)];
verty = [yeqn1(th1); yeqn2(th1); yeqn2(th2); yeqn1(th2); yeqn1(th1)];
vertz = [th1; th1; th2; th2; th1];
p2 = patch(vertx, verty, vertz, 'b')

vertx = [xeqn2(th1); xeqn1(th1); xeqn1(th2); xeqn2(th2); xeqn2(th1)];
verty = [yeqn2(th1); yeqn2(th1); yeqn2(th2); yeqn2(th2); yeqn2(th1)];
vertz = [th1; th1; th2; th2; th1];
p3 = patch(vertx, verty, vertz, 'g')

vertx = [xeqn1(th1); xeqn1(th1); xeqn1(th2); xeqn1(th2); xeqn1(th1)];
verty = [yeqn2(th1); yeqn1(th1); yeqn1(th2); yeqn2(th2); yeqn2(th1)];
vertz = [th1; th1; th2; th2; th1];
p4 = patch(vertx, verty, vertz, 'y')
p1.FaceAlpha = 0.7;
p2.FaceAlpha = 0.7;
p3.FaceAlpha = 0.7;
p4.FaceAlpha = 0.7;
p1.LineStyle = 'none';
p2.LineStyle = 'none';
p3.LineStyle = 'none';
p4.LineStyle = 'none';
hold off;
end

end



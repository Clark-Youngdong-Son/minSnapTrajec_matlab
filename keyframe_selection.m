%% Keyframe selection from mouse point

%Yaw is fixed to zero for all keyframes.


%Plots initialization
figure(1);
subplot(2,2,1);                 %x-y view
grid on; hold on;
xlabel('xy plane');
axis([-5 20 -12.5 12.5]);

subplot(2,2,2);                 %altitude view
set( gca, 'XTick', -1:1:1 );
set( gca, 'XTickLabel', {'','',''} );
grid on; hold on;
xlabel('altitude');
axis([-1 1 0 5]);

subplot(2,2,[3 4]);             %3-dimensional view
grid on; hold on;
xlabel('3D view');
axis([-5 20 -12.5 12.5 0 5]);

%Get data from mouse input
global count keyframe;
count = 1;
keyframe = zeros(4,m);
keyframe(:,1) = zeros(4,1);     %initial position and yaw

%xy selection
subplot(2,2,1);
plot(keyframe(1,count), keyframe(2,count), 'ro','MarkerFaceColor', 'r');
text(keyframe(1,count)+0.6, keyframe(2,count)+0.6, num2str(count));
set(gcf, 'WindowButtonMotionFcn', @mouseMove1);
set(gcf, 'WindowButtonDownFcn', @mouseClick1);
while(count~=m)
    drawnow;
end
title(gca,'Done. Select the desired altitudes');

%altitude selection
count = 1;
subplot(2,2,2);
plot([-1 1],[0 0], 'r-', 'LineWidth', 2);
text(0.6, 0.6, num2str(count));
set(gcf, 'WindowButtonMotionFcn', @mouseMove2);
set(gcf, 'WindowButtonDownFcn', @mouseClick2);
while(count~=m)
   drawnow; 
end
title(gca,'All selections done');

%3D view
subplot(2,2,[3 4]);
for i=1:m
   plot3(keyframe(1,i),keyframe(2,i),keyframe(3,i),'-ro','MarkerFaceColor','r');
   text(keyframe(1,i)+0.4, keyframe(2,i)+0.4, keyframe(3,i)+0.4, num2str(i));
end
view([-30.4000 45.2000]);
drawnow;
x_trajec   = [];
y_trajec   = [];
z_trajec   = [];
psi_trajec = [];
time = [];
for i=1:m
    x_trajec   = [x_trajec   polyval(solution((i-1)*n*(order+1)+1+0*(order+1):(i-1)*n*(order+1)+(order+1)+0*(order+1)),t(i):0.01:t(i+1))];
    y_trajec   = [y_trajec   polyval(solution((i-1)*n*(order+1)+1+1*(order+1):(i-1)*n*(order+1)+(order+1)+1*(order+1)),t(i):0.01:t(i+1))];
    z_trajec   = [z_trajec   polyval(solution((i-1)*n*(order+1)+1+2*(order+1):(i-1)*n*(order+1)+(order+1)+2*(order+1)),t(i):0.01:t(i+1))];
    psi_trajec = [psi_trajec polyval(solution((i-1)*n*(order+1)+1+3*(order+1):(i-1)*n*(order+1)+(order+1)+3*(order+1)),t(i):0.01:t(i+1))];
    time       = [time t(i):0.01:t(i+1)];
end

figure(1);
subplot(2,2,[3 4]);
plot3(x_trajec,y_trajec,z_trajec,'-k');    

figure(2);
subplot(4,1,1); plot(time,x_trajec,'-k'); ylabel('x'); hold on;
subplot(4,1,2); plot(time,y_trajec,'-k'); ylabel('y'); hold on;
subplot(4,1,3); plot(time,z_trajec,'-k'); ylabel('z'); hold on;
subplot(4,1,4); plot(time,psi_trajec,'-k'); xlabel('time'); ylabel('\psi'); hold on;

for i=1:size(keyframe,2)-1
    subplot(4,1,1); plot(t(i+1),keyframe(1,i+1),'or','MarkerFaceColor','r');
    subplot(4,1,2); plot(t(i+1),keyframe(2,i+1),'or','MarkerFaceColor','r');
    subplot(4,1,3); plot(t(i+1),keyframe(3,i+1),'or','MarkerFaceColor','r');
    subplot(4,1,4); plot(t(i+1),keyframe(4,i+1),'or','MarkerFaceColor','r');
end
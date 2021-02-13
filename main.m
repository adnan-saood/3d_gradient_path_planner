
%% Generate some points

nrows = 400;
ncols = 600;
nhe = 200;

plotcon = 1;

obstacle = false(nrows, ncols, nhe);

[x, y ,z] = meshgrid (1:ncols, 1:nrows, 1:nhe);

%% Generate some obstacle

 obstacle (300:end, 100:250, 20:90) = true;
 obstacle (150:200, 400:500 ,20:180) = true;
 obstacle (100:300, 100:300 ,1:40) = true;
 obstacle (100:300, 50:400 ,120:200) = true;

%% Compute distance transform

d = bwdist(obstacle);

% Rescale and transform distances

d2 = (d/100) + 1;

d0 = 2;
nu = 50;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;




%% Compute attractive force

goal = [400, 50, 190];

xi = 1/7;

attractive = xi * sqrt( (x - goal(1)).^2 + (y - goal(2)).^2 + (z - goal(3)).^2 );


figure(3);
plot3 (goal(1), goal(2) , goal(3), 'r.', 'MarkerSize', 25);
hold off;



title ('Configuration Space');

%% Combine terms

f = attractive + repulsive;



%% Plan route
start = [50, 350, 1];

route = GradientBasedPlanner3 (f, start, goal, 1000);

%% Plot the energy surface
if(plotcon)
    %%
    for i = 1:10
        subplot(2,5,i);
        contourf(f(:,:,i*5),30)
        axis equal
    end

end

%% quiver plot
[gx, gy ,gz] = gradient (-f);
skip = 20;

figure(6);

xidx = 1:skip:ncols;
yidx = 1:skip:nrows;
zidx = 1:skip:nhe;

 quiver3 (x(yidx,xidx,zidx), y(yidx,xidx,zidx),....
     z(yidx,xidx,zidx), gx(yidx,xidx,zidx), gy(yidx,xidx,zidx),gz(yidx,xidx,zidx) ,2 , 'Color', [0.3,0.1,0.1]);



hold on;

ps = plot3(start(1), start(2),start(3), 'r.', 'MarkerSize', 30);
pg = plot3(goal(1), goal(2),goal(3), 'g.', 'MarkerSize', 30);
p3 = plot3 (route(:,1), route(:,2),route(:,3), 'r', 'LineWidth', 2);

plotcube([150 100 70],[ 100  300  20],.7,[0.1 0.1 0.1]);
plotcube([100 50 160],[ 400  150  20],.7,[0.1 0.1 0.1]);
plotcube([200 200 40],[ 100  100  01],.7,[.1 0.10 .1]);
plotcube([350 200 80],[ 50  100  120],.7,[0.1 0.1 0.1]);
axis equal
%[300 100 20],[ 400  250  30]

% obstacle (300:end, 100:250, 20:30) = true;
% obstacle (150:200, 400:500 ,20:30) = true;
% obstacle (100:300, 100:300 ,1:190) = true;

% obstacle (300:end, 100:250, 20:90) = true;
%  obstacle (150:200, 400:500 ,20:180) = true;
%  obstacle (100:300, 100:300 ,1:40) = true;
%obstacle (100:300, 50:400 ,120:200) = true;





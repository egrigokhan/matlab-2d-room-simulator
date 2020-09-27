clear all
%% READ IMAGE

floorPlan = imread("fp_2.png");
floorPlanBW = ~imbinarize(rgb2gray(floorPlan));

rotI = floorPlanBW;
BW = edge(rotI,'canny');

% try
%     [co,bi] = imhist(floorPlanBW);
%     
%     if bi(1) == 0 && co(1) > co(2)
%         floorPlanBW = ~floorPlanBW;
%     end
% catch
% end

% HOUGH TRANSFORM
[H,T,R] = hough(BW);
imshow(H,[],'XData',T,'YData',R,...
            'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;

P  = houghpeaks(H,6,'threshold',ceil(0.2*max(H(:))));
x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

lines = houghlines(BW,T,R,P,'FillGap',10,'MinLength',5);
figure, imshow(rotI), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','cyan');
%% POINT CLOUD SAMPLING

pc = [];
for i = 1:length(lines)
    pc = [pc SampleNPointsFromLine(lines(i), 10)'];
end

plot(pc(1, :),pc(2, :),'x','LineWidth',2,'Color','magenta');

%% RGB-D

CP = [200 400; 350 300]; % CAMERA POSITIONS (x, y)

plot(CP(:, 1),CP(:, 2),'x','LineWidth',2,'Color','yellow');

CN = [-1 0.5; 1 1]; % CAMERA NORMALS / DIRECTIONS (n_x, n_y)

quiver(CP(:, 1),CP(:, 2),100 * CN(:, 1),100 * CN(:, 2),0)
%%
f = 20; % FOCAL DISTANCE
L = 20; % IMAGE PLANE WIDTH
N = 1000; % NUMBER OF SAMPLES ON EACH IMAGE PLANE
MAX_DIST = 1000;

RGBD = [];

% ITERATE THROUGH CP
for i = 1:length(CP)
    
    img = [];
    
    cp = CP(i, :);
    cn = CN(i, :);
    
    sp = SamplePointsOnImagePlaneForCamera(cp, cn, f, L, N); % SAMPLE POINTS ON IMAGE PLANE
    
    quiver(sp(1, 1),sp(2, 1),100 * (sp(1, 1) - cp(1)),100 * (sp(2, 1) - cp(2)),0)
    quiver(sp(1, round(N/3)),sp(2, round(N/3)),100 * (sp(1, round(N/3)) - cp(1)),100 * (sp(2, round(N/3)) - cp(2)),0)
    quiver(sp(1, round(2*N/3)),sp(2, round(2*N/3)),100 * (sp(1, round(2*N/3)) - cp(1)),100 * (sp(2, round(2*N/3)) - cp(2)),0)
    quiver(sp(1, N),sp(2, N),100 * (sp(1, N) - cp(1)),100 * (sp(2, N) - cp(2)),0)
    
    plot([sp(1, 1) sp(1, N)],[sp(2, 1) sp(2, N)], 'LineWidth',2,'Color','white');
    
    % ITERATE THROUGH SAMPLED POINTS
    for j = 1:length(sp)
        
        sp_ = sp(:, j);
        dist = [];
        
        % ITERATE THROUGH lines
        for k = 1:length(lines)
            dist = [dist Intersect(cn, cp, sp_ - cp', lines(k).point1, lines(k).point2, MAX_DIST)];
        end
        
        img = [img min(dist)];
        
    end
    
    RGBD = [RGBD img'];
end
%% LINE INTERSECTION

function dist = Intersect(cn, p11, d, p21, p22, max_dist)
    m1 = d(2) / (d(1)  + 1e-6);
    c1 = p11(2) - p11(1) * m1;
    
    m2 = (p22(2) - p21(2)) / (p22(1) - p21(1) + 1e-6);
    c2 = p21(2) - p21(1) * m2;
    
    xi = -1 * (c2 - c1) / (m2 - m1);
    yi = c1 + m1 * xi;

    if ((xi - p22(1))*(xi - p21(1)) <= 1e-6 && (yi - p22(2))*(yi - p21(2)) <= 1e-6)
        if((xi - p11(1))*cn(1) + (yi - p11(2))*cn(2) >= 1e-6)
            dist = sqrt((xi - p11(1))^2 + (yi - p11(2))^2);
            if(dist > max_dist)
                dist = max_dist + 1;
%             else
%                 plot(xi,yi,'o','LineWidth',2,'Color','red');
            end
        else
            dist = max_dist + 1;
        end
    else
        dist = max_dist + 1;
    end
end

%%
function sp = SamplePointsOnImagePlaneForCamera(pc, n, d, l, N)
    % CENTER OF IMAGE PLANE
    m = n(2) / (n(1) + 1e-6);
    k = d / sqrt(1 + m^2);
    
    pip = pc + [sign(n(1))*k sign(n(2))*abs(m)*k]
    
    % TANGENT LINE FOR IMAGE PLANE
    t = [-n(2) n(1)];
    
    mt = -1 * n(1) / (n(2) + 1e-6);
    kt = (l / 2) / sqrt(1 + mt^2);
    
    pip1 = pip + kt * [sign(-n(2)) sign(n(1))*abs(n(1) / (n(2) + 1e-6))];
    pip2 = pip - kt * [sign(-n(2)) sign(n(1))*abs(n(1) / (n(2) + 1e-6))];
    
    x_ls = linspace(pip1(1), pip2(1), N);
    y_ls = linspace(pip1(2), pip2(2), N);
    
    sp = [x_ls' y_ls']';
end

%%

function pc = SampleNPointsFromLine(line, N)
    t = rand(N, 1);
    pc = repmat(line.point1, N, 1) + t * [(line.point2(1) - line.point1(1)) (line.point2(2) - line.point1(2))];
end
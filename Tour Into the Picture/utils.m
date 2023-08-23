classdef utils
    methods (Static)

        function [x,y] = findCorner(vx,vy,rx,ry,limitx,limity)
            % This function takes the coordinates of the vanishing point, one of the rectangles and image limits and finds the intersection betweeen
            % all wall's lines and the image border
            y1 = limity;
            x1 = utils.findLineX(vx,vy,rx,ry,limity);
            x2 = limitx;
            y2 = utils.findLineY(vx,vy,rx,ry,limitx);
            if (sum(([vx vy]-[x1 y1]).^2) > sum(([vx vy]-[x2 y2]).^2))
              x = x1;
              y = y1;
            else
              x = x2;
              y = y2;
            end

        end

        function x = findLineX(x1,y1,x2,y2,y)
            % Given a line and a y coordinate, this function finds the matching x coordinate at the line 
            m = (y1-y2)./(x1-x2);
            b = y1 - m*x1;
            x = (y-b)/m;
            
        end

        function y = findLineY(x1,y1,x2,y2,x)
           % Given a line and a x coordinate, this function finds the matching y coordinate at the line 
           m = (y1-y2)./(x1-x2);
           b = y1 - m*x1;
           y = m*x + b;
            
        end

        function [big_im,big_im_alpha,vx,vy,ceilrx,ceilry,floorrx,floorry,leftrx,leftry,rightrx,rightry,backrx,backry, irx, iry] = get5rects(im,vx,vy,irx,iry,orx,ory)
            % This function computes all 5 faces for the scene. It also converts the given coordinates to the expanded scene with the drawed lines,
            % Returning them accordingly

            % Expand the image so that each "face" of the box is a proper rectangle
            [ymax,xmax,cdepth] = size(im);
            lmargin = -min(orx);
            rmargin = max(orx) - xmax;
            tmargin = -min(ory);
            bmargin = max(ory) - ymax;
            big_im = zeros([ymax+tmargin+bmargin xmax+lmargin+rmargin cdepth]);
            big_im_alpha = zeros([size(big_im,1) size(big_im,2)]);
            big_im(tmargin+1:end-bmargin,lmargin+1:end-rmargin,:) = im2double(im);
            big_im_alpha(tmargin+1:end-bmargin,lmargin+1:end-rmargin) = 1;
            
            
            % Update all variables for the new image
            vx = vx + lmargin;
            vy = vy + tmargin;
            irx = irx + lmargin;
            iry = iry + tmargin;
            orx = orx + lmargin;
            ory = ory + tmargin;
            
            
            %%% Defining the 5 rectangle faces %%%
            
            % Ceiling
            ceilrx = [orx(1) orx(2) irx(2) irx(1)];
            ceilry = [ory(1) ory(2) iry(2) iry(1)];
            if (ceilry(1) < ceilry(2))
                 ceilrx(1) = round(utils.findLineX(vx,vy,ceilrx(1),ceilry(1),ceilry(2)));
                 ceilry(1) = ceilry(2);
            else
                 ceilrx(2) = round(utils.findLineX(vx,vy,ceilrx(2),ceilry(2),ceilry(1)));
                 ceilry(2) = ceilry(1);
            end
            
            % Floor
            floorrx = [irx(4) irx(3) orx(3) orx(4)];
            floorry = [iry(4) iry(3) ory(3) ory(4)];
            if (floorry(3) > floorry(4))
                 floorrx(3) = round(utils.findLineX(vx,vy,floorrx(3),floorry(3),floorry(4)));
                 floorry(3) = floorry(4);
            else
                 floorrx(4) = round(utils.findLineX(vx,vy,floorrx(4),floorry(4),floorry(3)));
                 floorry(4) = floorry(3);
            end
            
            % Left
            leftrx = [orx(1) irx(1) irx(4) orx(4)];
            leftry = [ory(1) iry(1) iry(4) ory(4)];
            if (leftrx(1) < leftrx(4))
                 leftry(1) = round(utils.findLineY(vx,vy,leftrx(1),leftry(1),leftrx(4)));
                 leftrx(1) = leftrx(4);
            else
                 leftry(4) = round(utils.findLineY(vx,vy,leftrx(4),leftry(4),leftrx(1)));
                 leftrx(4) = leftrx(1);
            end
            
            % Right
            rightrx = [irx(2) orx(2) orx(3) irx(3)];
            rightry = [iry(2) ory(2) ory(3) iry(3)];
            if (rightrx(2) > rightrx(3))
                 rightry(2) = round(utils.findLineY(vx,vy,rightrx(2),rightry(2),rightrx(3)));
                 rightrx(2) = rightrx(3);
            else
                 rightry(3) = round(utils.findLineY(vx,vy,rightrx(3),rightry(3),rightrx(2)));
                 rightrx(3) = rightrx(2);
            end
            
            backrx = irx;
            backry = iry;

        end

        function createScene(back_image, right_wall, left_wall, ceiling, floor, width, depth, height)
            % Given the warped 5 images of the 5 faces and estimates of the box dimensions, this function draws the entire 3D scene.

            Xb = [0 width; 0 width];
            Yb = [height height ; 0 0];
            Zb = [-depth -depth; -depth -depth];

            % Create the surface and texturemap it 
            f = figure('Name', 'Constructed 3D scene', 'NumberTitle', 'off');
            set(gcf, 'Name', 'Constructed 3D scene');
            tb = cameratoolbar(f);
            cameratoolbar(f, 'SetMode', 'orbit')
            
            surf(Xb,Yb,Zb,'CData',back_image,'FaceColor','texturemap');
            
            axis on;
            set(gca,'DataAspectRatio',[width, height, depth/2.5])
            grid off;
            set(gca, 'color', 'none');
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            hold on;

            % Right Wall
            Xr = [width width; width width];
            Yr = [height height ; 0 0];
            Zr = [-depth 0; -depth 0];
            surf(Xr,Yr,Zr,'CData',right_wall,'FaceColor','texturemap');
            hold on;
            
            % Left Wall
            Xl = [0 0; 0 0];
            Yl = [height height ; 0 0];
            Zl = [0 -depth; 0 -depth];
            surf(Xl,Yl,Zl,'CData',left_wall,'FaceColor','texturemap');
            hold on;
            
            % Ceiling
            Xc = [0 width; 0 width];
            Yc = [height height; height height];
            Zc = [0 0; -depth -depth];
            surf(Xc,Yc,Zc,'CData',ceiling,'FaceColor','texturemap');
            hold on;
            
            % Floor
            Xf = [0 width; 0 width];
            Yf = [0 0; 0 0];
            Zf = [-depth -depth; 0 0];
            surf(Xf,Yf,Zf,'CData',floor,'FaceColor','texturemap');
            hold on;
            
            cameratoolbar(f,'SetCoordSys', 'y')
            camproj('perspective');
            figure(f);
            view(2);

            axis off;

        end

        function f = estimateFocalLength(irx, iry, vx, vy)
            % Given the coordinates of the inner rectangle and the vanishing point, this function esimates the focal length of the virtual camera.
            a = iry(3) - vy;
            b = vy - iry(2);
            f = (a+b)/a;
        end

        function depth = estimateDepth(irx, iry, vx, vy, xmax, ymax, f)
            % Given the coordinates of the inner rectangle, vanishing point, image dimensions and focal length,
            % this function estimates the depth of the scene by using triangle similarity.

            % Dbottom
            a = iry(3) - vy;
            h = ymax - vy;
            Dbottom = f * h / a - f;

            % Dtop
            b = vy - iry(2);
            l = vy;
            Dtop = f * l / b - f;

            % Dright
            alpha = xmax - vx;
            k = irx(2) - vx;
            Dright = f * alpha / k - f;

            % Dleft
            d = vx - irx(1);
            beta = vx;
            Dleft = f * beta / d - f;

            depth = max([Dbottom, Dtop, Dleft, Dright]);

        end


        function H = estimateHomography(rx, ry, xlimit, ylimit)
            % Given the face characterized by rx and ry, this function estimates the homography matrix between both perspectives.
            moving_points = [rx' ry'];
            fixed_points = [0 0 ; xlimit 0 ; xlimit ylimit ; 0 ylimit];
            H = fitgeotrans(moving_points, fixed_points,'projective');
        end

        function [orx, ory, H] = plotSurfaces(vp, ir, img, fg)
            % Main function called from the UI.

            close all;

            [ymax,xmax,color]=size(img);
            
            % Finding all image corners, where lines intersect
            [ox,oy] = utils.findCorner(vp(1),vp(2),ir(1),ir(2),0,0);
            orx(1) = ox;  ory(1) = oy;
            [ox,oy] = utils.findCorner(vp(1),vp(2),ir(3),ir(2),xmax,0);
            orx(2) = ox;  ory(2) = oy;
            [ox,oy] = utils.findCorner(vp(1),vp(2),ir(3),ir(4),xmax,ymax);
            orx(3) = ox;  ory(3) = oy;
            [ox,oy] = utils.findCorner(vp(1),vp(2),ir(1),ir(4),0,ymax);
            orx(4) = ox;  ory(4) = oy;
            orx = round(orx);
            ory = round(ory);
            
           
            % If there are any foreground objects selected, apply image inpainting technique
            if ~isempty(fg)
                masks=zeros(size(img,1),size(img,2));
                for i=1:size(fg,2)
                    L = superpixels(img,1300);
                    mask=createMask(fg(i).roi);
                    masks=logical(masks+mask);   
                end
                img=inpaintExemplar(img,masks);
            end
            
            irx = [ir(1) ir(3) ir(3) ir(1)];
            iry = [ir(2) ir(2) ir(4) ir(4)];
            
            % Computing all 5 rectangles
            [bim,bim_alpha,vx,vy,ceilrx,ceilry,floorrx,floorry,leftrx,leftry,rightrx,rightry,backrx,backry, irx, iry] = utils.get5rects(img,vp(1),vp(2),irx,iry,orx,ory);

            %% Estimating homographies
            % Left wall
            H = utils.estimateHomography(leftrx, leftry, xmax, ymax);
            left_wall = imwarp(bim, H, 'OutputView',imref2d(size(img)));

            % Ceiling
            H = utils.estimateHomography(ceilrx, ceilry, xmax, ymax);
            ceiling = imwarp(bim, H, 'OutputView',imref2d(size(img)));            

            % Right wall
            H = utils.estimateHomography(rightrx, rightry, xmax, ymax);
            right_wall = imwarp(bim, H, 'OutputView',imref2d(size(img)));

            % Floor
            H = utils.estimateHomography(floorrx, floorry, xmax, ymax);
            floor = imwarp(bim, H, 'OutputView',imref2d(size(img)));

            % Background
            aspect_ratio = (max(backrx) - min(backrx)) / (max(backry) - min(backry));
            back_image = bim(min(backry):max(backry), min(backrx):max(backrx), :);

            % We set the height as 1 and make the width equal to the aspect ratio
            height = 1;
            width = aspect_ratio;
            
            % Creating scene
            f = utils.estimateFocalLength(irx, iry, vx, vy);
            depth = utils.estimateDepth(irx, iry, vx, vy, xmax, ymax, f);            
            utils.createScene(back_image, right_wall, left_wall, ceiling, floor, width, depth, height);

        end
    end
end

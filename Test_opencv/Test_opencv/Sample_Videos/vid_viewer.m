clc;
%clear all;
close all;

%Open Video
v = VideoReader('video.mov');

frame_counter = 0;
while hasFrame(v)
    video = readFrame(v);
    
    %HSV test
    hsv_image = rgb2hsv(video); 
    
    H = hsv_image(:,:,1);
    S = hsv_image(:,:,2);
    V = hsv_image(:,:,3);
    
    
    R=video(:,:,1);
    G=video(:,:,2);
    B=video(:,:,3);
    
    %laplacian Pyramid
    
    Rd = im2double(R);
    Gd = im2double(G);
    Bd = im2double(B);
    
    level = 3;
    Rlap = genPyr(Rd,'lap',level); % the Laplacian pyramid
    Glap = genPyr(Gd,'lap',level); % the Laplacian pyramid
    Blap = genPyr(Bd,'lap',level); % the Laplacian pyramid
    
    frame_counter = frame_counter+1;
end


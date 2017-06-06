clc;
clear all;
close all;

I = imread('testHSV.png');

V = I(:,:,1);
S = I(:,:,2);
H = I(:,:,3);
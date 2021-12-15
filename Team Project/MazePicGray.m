
I = imread('image4.jpg');
imshow(I)
Ig=rgb2gray(I);
figure;
imshow(Ig);
figure;
[outline_og,h]=imcontour(Ig,1);
data=contourdata(outline_og);
data_c=struct2cell(data);
u_data=data_c(3,1,:);
v_data=data_c(4,1,:);
[max_size, max_index] = max(cellfun('size', u_data, 1));
u=u_data{max_index};
v=v_data{max_index};
outline=[u';v'];
figure;
set(gca, 'YDir','reverse');hold on;
plot(outline(1,:),outline(2,:))

%%
Ig2 = Ig < 100;
rad_range = [70 100].*(size(I,1)/3456);
[centers,radii]=imfindcircles(Ig2,round(rad_range,0),'ObjectPolarity','bright', ...
    'Sensitivity',0.9); %m of 3456 pixels for 75 to 150
figure;
imshow(Ig2);
viscircles(centers, radii,'EdgeColor','b');
outline=round(outline,0);
save('Output.mat','outline','centers');
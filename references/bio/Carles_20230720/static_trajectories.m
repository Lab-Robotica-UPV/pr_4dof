clear
clc
close all

files = ["TrayCarte_mus1_Carles", "TrayCarte_mus7_Carles", "TrayCarte_mus14_Carles", "TrayPatas_mus1_Carles", "TrayPatas_mus7_Carles", "TrayPatas_mus14_Carles"];

for i=1:numel(files)
    file = load(strcat(files(i), ".txt"));
    sample = file(1,:);
    tray = ones(12000,4) .* sample;
    name_file_final = strcat(files(i),"_Estatica.txt");
    writematrix(tray, name_file_final, 'Delimiter', ' ');
end
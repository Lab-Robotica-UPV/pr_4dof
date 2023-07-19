clear
clc
close all

files = ["TrayCarte_genforce_Jose", "TrayCarte_mus1_Jose", "TrayCarte_mus7_Jose","TrayPatas_genforce_Jose", "TrayPatas_mus1_Jose", "TrayPatas_mus7_Jose"];

for i=1:numel(files)
    file = load(strcat(files(i), ".txt"));
    sample = file(1,:);
    tray = ones(12000,4) .* sample;
    name_file_final = strcat(files(i),"_Estatica.txt");
    writematrix(tray, name_file_final, 'Delimiter', ' ');
end
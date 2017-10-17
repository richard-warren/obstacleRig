function makeNoise(fs, length, ampPeakPeak)

    noise = rand(1, fs*length) * ampPeakPeak;
    noise = noise - (ampPeakPeak/2);

    dlmwrite(['C:\Users\sawtell\Desktop\github\obstacleRig\matlab\noiseFs' num2str(fs)...
        'Length' num2str(length) '.txt'], noise', 'delimiter', '\t');
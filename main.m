%% Initializing Constants, Radar Parameters, Targets, and Results Structures 

% Include White Noise in the Model
noiseToggle = true;		% 0 - None, 1 - Yup

% Setup constants using Matlab's higher precision numbers
c  = physconst('LightSpeed');	% Speed of Ligh			- Meters/Second (m/s)
kb = physconst('Boltzmann');	% Boltzman Constant	- Joul/Kelvin (J/K)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radar Parameters   
Radar.TransmitPower					= 1*10^6;										% Transmit Power								- Watts (W)
Radar.MaxGainTx							= 30;												% Transmit Antenna Gain					- Decibles (dB)
Radar.MaxGainRx							= 30;												% Receive Antenna Gain					- Decibles (dB)
Radar.CenterFrequency				= 5*10^9;								    % Cenrer Frequency							- Hertz (Hz)
Radar.CenterWavelength			= c/Radar.CenterFrequency;	% Center Wavelength							- Meters (m)
Radar.Bandwidth							= 5*10^6;										% LFM Bandwidth									- Hertz (Hz)
Radar.SampleTime						= 1/24e6;										% Time Resolution								- seconds (s)
Radar.PRI										= Radar.SampleTime * 256;		% Pulse Rep Interval							- Seconds (s)
Radar.PRF										= 1/Radar.PRI;							% Pulse Rep Frequency						- Hertz (Hz)
Radar.Tau										= Radar.PRI / 127;					% Samples per PRI								- Seconds (s)
Radar.InputNoiseTemperature	= 270;											% Noise Temp										- Kelvin(K)
Radar.NoiseFigure						= 7;												% Noise Figure									- Decibles (dB)
Radar.SystemLosses					= 3;												% System Loss										- Decibles (dB)
Radar.AmbigousRange					= c/(Radar.PRF * 2);				% Ambigous Range								- Meters(m)
Radar.PulseDuration					= 3 * Radar.Tau;						% Pulse Duration							- Seconds (s)
Radar.NumberOfPulses				= 255;											% Number of Pulses							- Index (n)
Radar.WaveformType					= 0;												% Waveform Type: 0 - Simple Pulse, 1 - ILFM


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Antenna Parameters
Radar.ArrayDimX		= 10;													% Number of elements in the X-Dim
Radar.ArrayDimY		= Radar.ArrayDimX*2;					% Number of elements in the Y-Dim, multipled by two to account for the triangular spacing
Radar.dx					= 28.45e-3;										% Spacing between elements in the X-Dim
Radar.dy					= 32.77e-3/2;									% Spacing between elements in the Y-Dim, divided by two to account for the triangular spacing
Radar.Us					= 0;													% The beam steering X position
Radar.Vs					= 0;													% The beam steering Y position
Radar.thetaPnt		= 0;													% The pointing angle from the X-Axis (Ground) - Degrees
Radar.phiPnt			= 0;													% The pointing angle in the YZ axis					- Degrees
Radar.taperType		= 0;													% The type of Array Illumination
Radar.ScanAngle		= 1;													% Number of Scan Angles
Radar.Amn					= ElementIllumination(Radar);	% The Array Illumination Array


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Target Strucute(s)
Target(1).IntPos	= [2e4,0,0];			% Initial Position [x, y, z] (m)
Target(2).IntPos	= [2e4,0,0];			% Initial Position [x, y, z] (m)
Target(3).IntPos	= [1e4,0,0];			% Initial Position [x, y, z] (m)

Target(1).Pos = Target(1).IntPos;	% Updated Position [x, y, z] (m)
Target(2).Pos = Target(2).IntPos;	% Updated Position [x, y, z] (m)
Target(3).Pos = Target(3).IntPos;	% Updated Position [x, y, z] (m)

Target(1).Velocity	= [-500,500,0];		% Velocity [x, y, z] (m/s)
Target(2).Velocity	= [-30,0,0];		% Velocity [x, y, z] (m/s)
Target(3).Velocity	= [200,0,0];		% Velocity [x, y, z] (m/s)

Target(1).RCS			= db2pow(30);	% RCS (dBsm)
Target(2).RCS			= db2pow(30);	% RCS (dBsm)
Target(3).RCS			= db2pow(30);	% RCS (dBsm)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Pulse Train Model

% Single Pulse and Wait Time
TimePRI = 0:Radar.SampleTime: Radar.PRI - Radar.SampleTime;

% The time covered by n Pulse and Wait Intervals
TimeCPI = 0:Radar.SampleTime:Radar.NumberOfPulses*Radar.PRI - Radar.SampleTime;

if Radar.WaveformType == 0
    [TxWaveform, TxWaveformInPRI]				= SimplePulse(TimePRI, TimeCPI, Radar, 1);
elseif Radar.WaveformType == 1
	[TxWaveform, TxWaveformInPRI, ILFM]	= slowTimeLFMWaveform(TimePRI, TimeCPI, Radar, 1);
end
          
% Modeling the Pules Reflecting off the Target(s)
[TimeCPI, SignalRx, Target, RadarPerformanceReport] = PulseTrainRxSim(Radar, Target, TxWaveform, noiseToggle, true);


%% Important Functions

function [TimeCPI, SignalRx, Target, Results] = PulseTrainRxSim(Radar, Target, TxWaveform, noiseToggle, prvdPlts)

	% Create the Scattering Plot
	if prvdPlts == true
		% Target Pos Plots
		figure;
		sc = scatter3(0, 0, 0, ...
			'Marker', 'x',...
			'LineWidth', 1.5,....
			'SizeData', 200,....
			'MarkerEdgeColor','k'); 
		ax = gca;
		hold(ax, 'on');
		xlabel(ax,'X-Dim (km)','FontSize',14)
		ylabel(ax,'Y-Dim (km)','FontSize',14)
		zlabel(ax,'Z-Dim (km)','FontSize',14)

		fig2 = figure;
		for TgtCount = 1:length(Target)
			sb(TgtCount) = subplot(length(Target),1,TgtCount);
		end
	end

	% Setup constants using Matlab's higher precision numbers
	c  = physconst('LightSpeed');
	kb = physconst('Boltzmann');

	% Operational Wavelength
	lambda = c/Radar.CenterFrequency;

	% Initialize Results Structure
	for targetCount = 1:length(Target)
    	Results(targetCount).TrueRange					= 0;
    	Results(targetCount).MeasuredRange			= 0;
    	Results(targetCount).FastTimeLocation			= 0;
    	Results(targetCount).Phase							= 0;
    	Results(targetCount).TrueDopplerFreq			= 0;
    	Results(targetCount).MeasuredDopplerFreq	= 0;
    	Results(targetCount).Amplitude.Sum				= 0;
		Results(targetCount).Amplitude.AzDiff			= 0;
		Results(targetCount).Amplitude.ElDiff			= 0;
    	Results(targetCount).PowerRecieved				= 0;
    	Results(targetCount).NoisePower					= 0;
    	Results(targetCount).SignalToNoiseRatio		= 0;
	end

	% The number of samples inside one CPI, Lecture 7 pg 12
	LengthCPI = floor(Radar.PRI/Radar.SampleTime);

	% The number of pulses
	numberPulses = Radar.NumberOfPulses;

	% Total Time Vector across the entire CPI
	TimeCPI = 0:Radar.SampleTime:numberPulses*Radar.PRI - Radar.SampleTime;
	
	% Creating the Received Signal Object
	SignalRx.Sum		= zeros(length(Target),length(TimeCPI), Radar.ScanAngle);
	SignalRx.AzDiff		= zeros(length(Target),length(TimeCPI), Radar.ScanAngle);
	SignalRx.ElDiff		= zeros(length(Target),length(TimeCPI), Radar.ScanAngle);

	% Generate Rx Noisee
	if noiseToggle == true
    	NoisePower = pow2db(kb * (1/Radar.SampleTime) * db2pow(Radar.NoiseFigure) * Radar.InputNoiseTemperature);

		Nse	= (randn(Radar.ScanAngle,LengthCPI*numberPulses) + 1i*randn(1,LengthCPI*numberPulses))*sqrt(db2pow(NoisePower)/2);
	end

	% Looping thorugh the number of targets
	for targetCount = 1:length(Target)
		
		% Looping though the Pulses in a search angle
		for pulseCount = 1:numberPulses
			
			% Target Position and Range
			[Target(targetCount).Pos, Results(targetCount).TrueRange(pulseCount)] = UpdateTargetPosition( Target(targetCount).IntPos, Target(targetCount).Velocity, ((pulseCount - 1) * Radar.PRI));
			
			% True Phase Shift
			Results(targetCount).Phase(pulseCount) = (mod((2*Results(targetCount).TrueRange(pulseCount)*2*pi)/lambda,2*pi));
			
			% True Doppler Frequency
			Results(targetCount).TrueDopplerFreq(pulseCount) = DopplerFrequency(Target(targetCount).Pos, Target(targetCount).Velocity, Radar.CenterFrequency);
			
			% Monopulse Antenna Gain
			[Gt, Gr] = MonopulseAntennaGain(Target(targetCount).Pos, Radar.thetaPnt, Radar.phiPnt, Radar.dx, Radar.dy, Radar.Us, Radar.Vs, ...
												  Radar.Amn, Radar.ArrayDimX, Radar.ArrayDimY, Radar.MaxGainTx, Radar.MaxGainRx,  lambda);
			
			% Monopulse Power Received
			[Pr] = CalculateMonopulsePowerReceived(Radar.TransmitPower, Gt, Gr, lambda, Target(targetCount).RCS, Radar.SystemLosses, Results(targetCount).TrueRange(pulseCount));

			Results(targetCount).PowerRecievedSum(pulseCount) = Pr.Sum;
			Results(targetCount).PowerRecievedAzDiff(pulseCount) = Pr.AzDiff;
			Results(targetCount).PowerRecievedElDiff(pulseCount) = Pr.ElDiff;

			% Amplitude Received
			Results(targetCount).Amplitude(pulseCount).Sum	= sqrt(db2pow( Pr.Sum ));    
			Results(targetCount).Amplitude(pulseCount).AzDiff	= sqrt(db2pow( Pr.AzDiff ));
			Results(targetCount).Amplitude(pulseCount).ElDiff	= sqrt(db2pow( Pr.ElDiff ));

			% Signal to Noise Ratio (dB)
			Results(targetCount).SignalToNoiseRatio(pulseCount) = SignalToNoiseRatio(Radar.TransmitPower, Gt.Sum, Gr.Sum, lambda, Target(targetCount).RCS,...
																									Radar.InputNoiseTemperature, Radar.Bandwidth, Radar.NoiseFigure, Radar.SystemLosses,...
																									Results(targetCount).TrueRange(pulseCount));
			
			% Generate Received Signals
			if (Radar.WaveformType == 0)
				SignalRx = GenerateMonopulseRxSignal(SignalRx, (pulseCount-1)*Radar.PRI, TimeCPI, Results(targetCount).TrueRange(pulseCount),...
									TxWaveform, Results(targetCount).Amplitude(pulseCount), Results(targetCount).TrueDopplerFreq(pulseCount), targetCount);
			elseif (Radar.WaveformType == 1)
				SignalRx = GenerateMonopulseRxSignal(SignalRx, (pulseCount-1)*Radar.PRI, TimeCPI, Results(targetCount).TrueRange(pulseCount),...
									TxWaveform(pulseCount,:), Results(targetCount).Amplitude(pulseCount), Results(targetCount).TrueDopplerFreq(pulseCount), targetCount);
			end
    			
		end
		
		% Plotting Target Position
		if prvdPlts == true
			scatter3(ax, Target(targetCount).Pos(1) ./ 10^3, Target(targetCount).Pos(2)  ./ 10^3, Target(targetCount).Pos(3)  ./ 10^3,...
							'LineWidth', 1.5,....
							'SizeData', 200,....
							'MarkerEdgeColor','k',...
							'MarkerFaceColor',[0 .75 .75]);
		end

	end

	% Adding Noise to the Received Signals
	if noiseToggle == true
		SignalRx.Sum = SignalRx.Sum + Nse;
		SignalRx.AzDiff = SignalRx.AzDiff + Nse;
		SignalRx.ElDiff = SignalRx.ElDiff + Nse;
	end

	%%%%%%%%%%%%%%%%%%%%%%%%
	% Plotting the Received Signals
	RxCPIPlot	= pow2db(abs(SignalRx.Sum(:,:,1)));	
	RxCPIPlot(RxCPIPlot == -Inf) = -200;

	if prvdPlts == true
		for targetCount = 1:length(Target)
		plot(sb(targetCount),TimeCPI.*10^6,RxCPIPlot(targetCount,:),'linewidth',1)
		title(sb(targetCount),"Siganl Return for Target " + targetCount)
		xlabel(sb(targetCount), 'Time (\mus)')
		ylabel(sb(targetCount), 'dB')
		xlim(sb(targetCount),[0 TimeCPI(end).*10^6])
		if noiseToggle == true
			ylim(sb(targetCount),[(pow2db(abs(max(Nse,[],'all'))) - 10) (max(RxCPIPlot,[],'all') + 5)])
		else
			ylim(sb(targetCount),[(max(RxCPIPlot(targetCount,:),[],'all') - 5) (max(RxCPIPlot(targetCount,:),[],'all') + 5)])
		end
		grid(sb(targetCount),"on");
		end
	end
	%%%%%%%%%%%%%%%%%%%%%%%%

	% Adding the Received Signals from each Target
	SignalRx.Sum	= sum(SignalRx.Sum(:,:),1);
	SignalRx.AzDiff	= sum(SignalRx.AzDiff(:,:),1);
	SignalRx.ElDiff	= sum(SignalRx.ElDiff(:,:),1);

end


function [Position, RadialRange] = UpdateTargetPosition(InitialPosition, Velocity, Time)	
% Returns a target Pos [x, y, z] and Radial Range given an Initial
% Position [x, y, z], Velocity [x, y, z], and Time Stamp (s)

	%%% Target Position %%%
	Position = InitialPosition + Velocity .* Time;
	
	%%% Radial Range %%%
	RadialRange = sqrt(sum(Position.^2));

end


function Doppler = DopplerFrequency(Position, Velocity, fc)
% Returns a target's doppler frequency (Hz) assuming the radar 
% is located at [0, 0, 0] and pointed in +X-Axis and given the Target's
% Position [x, y, z] and Velocity [x, y, z] and Radar Center Frequency (Hz)	

	c = physconst('lightspeed');

	% Calculating the target's radial velocity using vector projections
	velocityProjection	= -1.*(dot(Velocity , Position/norm(Position))); 

	Doppler					= ((2 * velocityProjection * fc)/(c));

end


function [Gt, Gr] = MonopulseAntennaGain(Position, faceAngleTheta, faceAnglePhi, elementSpacingX, elementSpacingY, beamSteerU, beamSteerV, ...
															  ElementAmplitudes, ArrayDimX, ArrayDimY, MaxGainTx, MaxGainRx,  lambda)
% Returns the Tx and Rx Gain for the given antenna parameters and
% Target parameters. Gt and Gr are objects that contain the Sum, 
% Azimuth, and Elevation gain values.
			
	thetaObs	= atan2(sqrt( Position(2)^2 + Position(3)^2 ), Position(1));		%
	phiObs		= atan2(Position(3), Position(2));											%
	
	theta	= thetaObs - deg2rad(faceAngleTheta);
	phi		= phiObs	- deg2rad(faceAnglePhi);
	
	u = sin(theta) .* cos(phi);													% Observation location in the X-Axis of the Y/Z Plane
	v = sin(theta) .* sin(phi);													% Observation location in the Y-Axis of the Y/Z Plane
	
	Txs = (2*pi / lambda) .* elementSpacingX .* beamSteerU;			% The element-element phase shifting for the given beam steering X position
	Tys = (2*pi / lambda) .* elementSpacingY .* beamSteerV;				% The element-element phase shifting for the given beam steering Y position
	
	[Sum, AzDiff, ElDiff] = TwoDimMonopulseArrayFactor(ElementAmplitudes, u, v, elementSpacingX, elementSpacingY, Txs, Tys, ArrayDimX, ArrayDimY, lambda, 0);
	
	Gt.Sum = Sum .* db2pow(MaxGainTx);
	Gr.Sum = Sum .* db2pow(MaxGainRx);

	Gt.AzDiff = abs(AzDiff) .* db2pow(MaxGainTx);
	Gr.AzDiff = abs(AzDiff) .* db2pow(MaxGainRx);

	Gt.ElDiff = abs(ElDiff) .* db2pow(MaxGainTx);
	Gr.ElDiff = abs(ElDiff) .* db2pow(MaxGainRx);
	
end


function [Pr] = CalculateMonopulsePowerReceived(Pt, Gt, Gr, lambda, RCS, Lsys, Range)
% Calculates the Power Received for a Monopulse System
	
	Pr.Sum = pow2db((Pt .* Gt.Sum .* Gr.Sum .* lambda.^2 .* RCS) ./ ((4*pi)^3 .* db2pow(Lsys) .* Range.^4));

	Pr.AzDiff = pow2db((Pt .* Gt.AzDiff .* Gr.AzDiff .* lambda.^2 .* RCS) ./ ((4*pi)^3 .*  db2pow(Lsys) .* Range.^4));

	Pr.ElDiff = pow2db((Pt .* Gt.ElDiff .* Gr.ElDiff .* lambda.^2 .* RCS) ./ ((4*pi)^3 .*  db2pow(Lsys) .* Range.^4));

end


function SNR = SignalToNoiseRatio(Pt, Gt, Gr, lambda, RCS, noiseTemperature, bandwidth, Fn, Lsys, Range)
% Calculates the Signal to Noise Ratio

	kb = physconst('Boltzmann');

	SNR = pow2db( (Pt .* Gt .* Gr .* lambda.^2 .* RCS) ./...
							  ((4*pi)^3 .* kb .* noiseTemperature .* bandwidth .* db2pow(Fn) .* db2pow(Lsys) .* Range.^4) ); 

end


function [SignalRx] = GenerateMonopulseRxSignal(SignalRx, TimePRI, TimeCPI, Range, TxWaveform, Amplitude, DopplerFreq, targetCount)

% Generates the Received Signals for a Monopulse Signals

	c = physconst('lightspeed');

	timeDelay  = TimePRI + ((2*Range)/c);
			
	%%% Finding the indecies for the recived waveform according to the type %%%
	indexWavefront = find((TimeCPI >= timeDelay));
	indexDelay = indexWavefront:indexWavefront+(size(TxWaveform,2)-1);
	
	%%% Scaling the Wave's Amplitude, Applying doppler shifting, and Placing the waveform onto the CPI timeline %%%
	if ((size(indexDelay,2) == size(TxWaveform,2) && (indexDelay(end) <= size(TimeCPI,2))))
		
		WaveformRx.Sum	= Amplitude.Sum .* exp(1i .* 2 .* pi .* DopplerFreq .* TimeCPI(indexDelay)) .* TxWaveform;
		WaveformRx.AzDiff = Amplitude.AzDiff .* exp(1i .* 2 .* pi .*DopplerFreq .* TimeCPI(indexDelay)) .* TxWaveform;
		WaveformRx.ElDiff = Amplitude.ElDiff .* exp(1i .* 2 .* pi .* DopplerFreq .* TimeCPI(indexDelay)) .* TxWaveform;

		SignalRx.Sum(targetCount,indexDelay)		= WaveformRx.Sum(1:length(indexDelay));
		SignalRx.AzDiff(targetCount,indexDelay)	= WaveformRx.AzDiff(1:length(indexDelay));
		SignalRx.ElDiff(targetCount,indexDelay)	= WaveformRx.ElDiff(1:length(indexDelay));
		
	end

end


function [Amn] = ElementIllumination(Radar)

	% Uniform Illumination
	if (Radar.taperType == 0)
	
		Amn = ones(Radar.ArrayDimY, Radar.ArrayDimX);
		Amn(1:2:Radar.ArrayDimY-1,1:2:Radar.ArrayDimX-1) = 0;
		Amn(2:2:Radar.ArrayDimY,2:2:Radar.ArrayDimX) = 0;
	
		Radar.taperTypeLabel = "Uniform";
	
	
	% Vertical Fan Illumination
	elseif (Radar.taperType == 1)
	
		Amn(2:2:Radar.ArrayDimY, 6:2:8) = 1;
	
		Radar.taperTypeLabel = "Vertical Fan Beam";
	
	
	% Cosine Tapering
	elseif (Radar.taperType == 2)
	
		padding = [0, 8];	% Set to 8 for a 32x32 array
	
		temp = cosineTaperTriangleSpacing(Radar.ArrayDimX - padding(1), Radar.ArrayDimY - padding(2));
	
		Amn = zeros(Radar.ArrayDimY, Radar.ArrayDimX);
		Amn(padding(2)/2 + 1:padding(2)/2 + size(temp,1),padding(1)/2 + 1: padding(1)/2 + size(temp,2)) = temp;
	
		Radar.taperTypeLabel = "Cosine";
	
	
	% Hanning Taper
	elseif (Radar.taperType == 3)
	
		padding = [0, 8];	% Set to [0, 8] for a 32x32 array to be symmetrical in Az and El
	
		temp = hannTriangleSpacing(Radar.ArrayDimX - padding(1), Radar.ArrayDimY - padding(2));
	
		Amn = zeros(Radar.ArrayDimY, Radar.ArrayDimX);
		Amn(padding(2)/2 + 1:padding(2)/2 + size(temp,1),padding(1)/2 + 1: padding(1)/2 + size(temp,2)) = temp;
		
		Radar.taperTypeLabel = "Hanning";
	
	
	% Symmetrical Uniform,	
	elseif (Radar.taperType == 4)
	
		Amn = ones(Radar.ArrayDimY, Radar.ArrayDimX);
		Amn(2:2:Radar.ArrayDimY,1:2:Radar.ArrayDimX-1) = 0;
		Amn(1:2:Radar.ArrayDimY-1,2:2:Radar.ArrayDimX) = 0;
		Amn(1:4, 1:Radar.ArrayDimX) = 0;
		Amn(Radar.ArrayDimY-3:Radar.ArrayDimY,1:Radar.ArrayDimX) = 0;
	
		Radar.taperTypeLabel = "Symmetrical Uniform";
	
	end

end


function [TxWaveform, TxSignal] = SimplePulse(TimePRI, TimeCPI, Radar, prvdPlts)

	TxSignal = zeros(1,length(TimeCPI));
	
	lengthPRI = length(TimePRI);
	lengthCPI = length(TimeCPI);
	PulseIndecies = find(TimePRI <= Radar.PulseDuration);
	
	for m = 1:Radar.NumberOfPulses
	
		TxSignal(PulseIndecies + ((m - 1) * lengthPRI)) = 1;
	
	end

	TxWaveform = ones(1,length(PulseIndecies));
	
	if prvdPlts == 1
		figure
		subplot(2,1,1)
		plot(TimeCPI.*10^6, TxSignal, 'linewidth', 1.5)
		xlim([0 Radar.PRI.*10^6])
		grid on;
		
		
		% Frequency Domain
		n = 2^nextpow2(lengthCPI);
		Ts = (TimeCPI(2) - TimeCPI(1)); Fs = 1/Ts;
		Freq = (-n/2 : n/2-1) * (Fs / n); 
		SignalFFT = fftshift(fft(TxSignal,n)) ./ n;	
		
		subplot(2,1,2)
		plot(Freq./10^3, abs(SignalFFT), 'linewidth', 1.5)
		grid on;
	end

end


function [TxWaveform, TxSignal, LFM] = slowTimeLFMWaveform(TimePRI, TimeCPI, Radar, prvdPlts)

	Signal = zeros(1,length(TimeCPI));
	
	lengthPRI = length(TimePRI);
	lengthCPI = length(TimeCPI);
	PulseIndecies = find(TimePRI <= Radar.PulseDuration);
	
	for m = 1:Radar.NumberOfPulses
	
		Signal(PulseIndecies + ((m - 1) * lengthPRI)) = 1;
	
	end
	
	%% LFM 
	
	LFM = exp(1i .* -pi .* (Radar.Bandwidth/TimeCPI(end)) .*  (0.5 .* TimeCPI(end) - TimeCPI).^2);
	
	if prvdPlts == 1
		figure
		subplot(2,1,1)
		plot(TimeCPI.*10^6, real(LFM), 'linewidth', 1.5)
		
		
		% Frequency Domain
		n = 2^nextpow2(lengthCPI);
		Ts = (TimeCPI(2) - TimeCPI(1)); Fs = 1/Ts;
		Freq = (-n/2 : n/2-1) * (Fs / n); 
		LFMFFT = fftshift(fft(LFM,n)) ./ n;	
		
		subplot(2,1,2)
		plot(Freq./10^6, abs(LFMFFT), 'linewidth', 1.5)
		grid on;
	end
	
	
	%% Combining Pulses and LFM
	
	TxSignal = Signal .* LFM;

	TxWaveform = zeros(Radar.NumberOfPulses, length(PulseIndecies));

	% Recording Each Pulse in the Tx Waveform Array
	for m = 1:Radar.NumberOfPulses
		TxWaveform(m,:) = TxSignal(PulseIndecies + ((m - 1) * lengthPRI));
	end
	
	if prvdPlts == 1
		figure
		subplot(2,1,1)
		plot(TimeCPI.*10^6, abs(TxSignal), 'linewidth', 1.5)
		
		
		% Frequency Domain
		n = 2^nextpow2(lengthCPI);
		Ts = (TimeCPI(2) - TimeCPI(1)); Fs = 1/Ts;
		Freq = (-n/2 : n/2-1) * (Fs / n); 
		TxSigFFT = fftshift(fft(TxSignal,n)) ./ n;	
		
		subplot(2,1,2)
		plot(Freq./10^6, abs(TxSigFFT), 'linewidth', 1.5)
		grid on;
	end

end


function [Sum, AzDiff, ElDiff] = TwoDimMonopulseArrayFactor(Amn, u, v, dx, dy, Txs, Tys, ArrayXDim, ArrayYDim, lambda, desiredPlots)

	%% Calculating the max AF

	Tx = 0;				% The phase difference between elements in the X-Dim due to position
	Ty = 0;				% The phase difference between elements in the Y-Dim due to position

	MaxAF = 0;
	for m = 1:ArrayXDim
		for n = 1:ArrayYDim
			MaxAF = MaxAF + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end

	%% Calculating the AF Sum, Az Diff, and El Diff at the Observation Point

	Tx = (2*pi / lambda) .* dx .* u;					% The phase difference between elements in the X-Dim due to position
	Ty = (2*pi / lambda) .* dy .* v.';				% The phase difference between elements in the Y-Dim due to position


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%% Sum Beamwidth
	Sum = zeros(length(v), length(u));
	for m = 1:ArrayXDim
		for n = 1:ArrayYDim
			Sum = Sum + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%% Azimuth DIfference
	AFLeft = zeros(length(v), length(u));
	% Iterate through the elements in the array
	for m = 1:ArrayXDim/2
		for n = 1:ArrayYDim
			AFLeft = AFLeft + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end

	AFRight = zeros(length(v), length(u));
	% Iterate through the elements in the array
	for m = ArrayXDim/2+1:ArrayXDim
		for n = 1:ArrayYDim
			AFRight = AFRight + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end

	AzDiff = AFLeft - AFRight;


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%% Elevation DIfference
	AFTop = zeros(length(v), length(u));
	% Iterate through the elements in the array
	for m = 1:ArrayXDim
		for n = 1:ArrayYDim/2
			AFTop = AFTop + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end

	AFBottom = zeros(length(v), length(u));
	% Iterate through the elements in the array
	for m = 1:ArrayXDim
		for n = ArrayYDim/2+1:ArrayYDim
			AFBottom = AFBottom + (  abs(Amn(n,m)) .* exp(1i.*( m.*(Tx - Txs) + n.*(Ty - Tys) ))  );
		end
	end
	
	ElDiff = AFTop - AFBottom;
	

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%%% Normalizing
	Sum		= abs(Sum./MaxAF);
	AzDiff	= abs(AzDiff./MaxAF);
	ElDiff	= abs(ElDiff./MaxAF);

	if u < 0
		AzDiff = AzDiff .* -1;
	end

	if v < 0
		ElDiff = ElDiff .* -1;
	end

end

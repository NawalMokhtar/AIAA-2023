%% Class Defintion

classdef Parasite_Drag < handle
    properties(Access = private)
        flow ; % Turbulance or other "Lamianr" 
        Sref;
        rho = 1.225;
        Vcr;
        viscosity = 1.789e-5;
    end
    methods 
        function obj = Parasite_Drag(Sreference,Vcr,flowType)
            obj.Sref = Sreference;
            obj.Vcr = Vcr;
            obj.flow = flowType;
        end
       
        %% wing_drag
        %TC is the thickness to chord ratio
        % XC is the max chord location
        % MAC-> mean aerodynamic chord which will be assumed to equal the
        % chord since the taper ratio = 1
        % outputs CD0 if the wing 

        function wing_CD0 = Wing(obj,TC,MAC,XC)
            Re = obj.Vcr*MAC*obj.rho/obj.viscosity;
            if obj.flow == 'T'
                Cf = 0.455/(log10(Re))^2.58;
            else
                Cf = 1.328/sqrt(Re);
            end
            FF = (1+0.6*TC/XC+100*TC^4);
            Swet = 2*(1+0.2*TC)*obj.Sref;
            wing_CD0 = Cf*FF*(Swet/obj.Sref);
        end
        %% Tails_drag

        % inputs are the same as the sing in addition to the area of the H
        % and V tails

            % Horizontal Tail
        function Htail_CD0 = Htail(obj,TC,SHtail,MAC,XC)
            Re = obj.Vcr*MAC*obj.rho/obj.viscosity;
            if obj.flow == 'T'
                Cf = 0.455/(log10(Re))^2.58;
            else
                Cf = 1.328/sqrt(Re);
            end
            FF = (1+0.6*TC/XC+100*TC^4);
            Swet = 2*(1+0.2*TC)*SHtail;
            Htail_CD0 = Cf*FF*(Swet/obj.Sref);
        end

             % Vertical Tail
        function Vtail_CD0 = Vtail(obj,TC,SVtail,MAC,XC)
            Re = obj.Vcr*MAC*obj.rho/obj.viscosity;
            if obj.flow =='T'
                Cf = 0.455/(log10(Re))^2.58;
            else
                Cf = 1.328/sqrt(Re);
            end
            FF = (1+0.6*TC/XC+100*TC^4);
            Swet = 2*(1+0.2*TC)*SVtail;
            Vtail_CD0 = Cf*FF*(Swet/obj.Sref);
        end
        %% Fuselage

        % A max is the frontal area 
        % Swet is aprroximated between 0.7:0.8 * the side area
        % inputs to this function are the fuselage length and diameter (IG
        % it is assumed to be cylinder) and the output is the CD0 of the
        % fuselge 

        function Fuselage_CD0 = Fuselage(obj,length,d)
            Re = obj.Vcr*length*obj.rho/obj.viscosity;
            if obj.flow =='T'
                Cf = 0.455/(log10(Re))^2.58;
            else
                Cf = 1.328/sqrt(Re);
            end
           Amax = 0.25*pi*d^2;
           fineness_ratio=length/sqrt(Amax*4/pi);
           FF=(1+60/fineness_ratio^3+fineness_ratio/400);
           Swet = 0.75*pi*length*d;
           Fuselage_CD0 = Cf*FF*(Swet/obj.Sref);
        end
       
    end
end

% Maybe add the landing gear friction later 

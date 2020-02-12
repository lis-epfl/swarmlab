classdef VideoWriterWithRate < handle
    
% VIDEO_WRITER _WITH_RATE - Create a video from matlab window, with a
% specified output rate
%
% Properties:
%   video
%   output_rate
%   time_of_last_frame
% 
% To create an object, call VideoWriterWithRate(video_path, output_rate).

    properties
        video
        output_rate
        time_of_last_frame
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        %%%%%%%%% Constructor %%%%%%%%%%%
        function self = VideoWriterWithRate(video_path, output_rate)
            self.video = VideoWriter(video_path,'Motion JPEG AVI');
            self.video.Quality = 95;
            open(self.video)
            self.output_rate = output_rate;
            self.time_of_last_frame = 0;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = update(self, time, figure_handle)
            if (time-self.time_of_last_frame >= self.output_rate && ...
                    ~isempty(figure_handle))
                % set(figure_handle,'Units','pixels','Position',[1196 424 580 480])
                frame = getframe(figure_handle);
                writeVideo(self.video, frame);
                self.time_of_last_frame = time;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = close(self)
            close(self.video);          
        end
    end
end
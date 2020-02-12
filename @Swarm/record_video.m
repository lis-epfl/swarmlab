function record_video(self, time, T, period, fig_handle, path)

    persistent video_handle

    % Open video
    if time == 0
        video_handle = VideoWriter(path);
        video_handle.Quality = 100;
        open(video_handle);

    elseif mod(time, period) == 0
        frame = getframe(fig_handle);
        writeVideo(video_handle, frame);
    end

    % Close video
    if time == T
        close(video_handle);
        close(fig_handle);
    end

end

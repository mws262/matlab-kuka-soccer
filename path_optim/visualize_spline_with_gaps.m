function spline_fig = visualize_spline_with_gaps(qp_spline, nlp_spline)

spline_fig = figure;
spline_fig.Color = [1,1,1];

%% Plot the QP version.
teval = linspace(qp_spline.breaks(1), qp_spline.breaks(end), 300);
qp_spline_eval = ppval(qp_spline, teval)';

subplot(1,3,1);
plot(qp_spline_eval(:,1), qp_spline_eval(:,2)); % Plot un-linearized version.
xlabel('qp spline');
daspect([1,1,1]);

%% Plot the NLP version.
subplot(1,3,2);
nlp_spline_eval = ppval(nlp_spline, teval)';
plot(nlp_spline_eval(:,1), nlp_spline_eval(:,2));
xlabel('nlp spline');
daspect([1,1,1]);

%% Vis non-contact sections.

% Split the fully piecewise polynomial into sections. Makes a separate
% piecewise polynomial, starting at time 0, for each contact region.
[zero_starts, zero_ends, contact_polys, shifted_pp] = find_zero_accel_breaks_from_pos_pp(nlp_spline, 3, 1e-3);

subplot(1,3,3);
% Plot the continuous parts of the contact areas.
hold on;
for i = 1:length(contact_polys)
   contact_section_eval = ppval(contact_polys(i), linspace(contact_polys(i).breaks(1), contact_polys(i).breaks(end), 100))';
   plot(contact_section_eval(:,1), contact_section_eval(:,2));
end

% Plot the boundaries as dots.
starts_eval = ppval(nlp_spline, zero_starts)'; % Starts is starts of non-contact, unintuitively.
ends_eval = ppval(nlp_spline, zero_ends)';
plot(starts_eval(:,1), starts_eval(:,2), '.r', 'MarkerSize', 10); % Plot just the contact areas beginning/ends.
plot(ends_eval(:,1), ends_eval(:,2), '.g', 'MarkerSize', 10);
xlabel('contact areas');
hold off;

daspect([1,1,1]);
end


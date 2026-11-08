classdef srekf_utils
    % Static mathematical and geodetic utility functions for SR-EKF (Core/EKF/Src/ekf_utils.c, compute_hats.c, and filter_pressure.c.)

    methods (Static)

        function C_bn = quaternion2dcm(q)
            % converts a unit quaternion [q0; q1; q2; q3] (scalar first)
            % into body-to-navigation Direction Cosine Matrix (C_b^n).
            % matches quaternion2dcm in ekf_utils.c
            q = q(:) / norm(q);
            s = q(1);
            v = q(2:4);

            v_cross = srekf_utils.skew(v);
            C_bn = (s^2 - (v' * v)) * eye(3) + 2 * (v * v') + 2 * s * v_cross;
        end

        function S = skew(v)
            % skew returns the 3x3 skew-symmetric cross-product matrix
            S = [    0, -v(3),  v(2);
                v(3),     0, -v(1);
                -v(2),  v(1),     0];
        end

        function q_prod = quat_mult(p, q)
            % quat_mult quaternion product p (x) q with scalar first
            sp = p(1); vp = p(2:4);
            sq = q(1); vq = q(2:4);

            s_out = sp * sq - dot(vp, vq);
            v_out = sp * vq + sq * vp + cross(vp, vq);
            q_prod = [s_out; v_out];
        end

        function radii = compute_radii(phi_deg, a, e2)
            % computes WGS-84 meridional and transverse radii of curvature
            % and their partial derivatives with respect to latitude
            % output: radii = [R_phi, R_lamb, dR_phi_dphi, dR_lamb_dphi]
            if nargin < 2, a = 6378137.0; end
            if nargin < 3, e2 = 1.0 - (6356752.31425 / a)^2; end

            phi = deg2rad(phi_deg);
            sinphi = sin(phi);
            cosphi = cos(phi);
            sqrtF = sqrt(1.0 - e2 * (sinphi^2));

            R_phi = a * (1.0 - e2) / (sqrtF^3);
            R_lamb = a / sqrtF;

            dR_phi_dphi = 3.0 * a * (1.0 - e2) * e2 * sinphi * cosphi / (sqrtF^5);
            dR_lamb_dphi = a * e2 * sinphi * cosphi / (sqrtF^3);

            radii = [R_phi, R_lamb, dR_phi_dphi, dR_lamb_dphi];
        end

        function [g, dg_dphi, dg_dh] = compute_gravity(phi_deg, h)
            % wgs-84 gravity model and derivatives
            % output: surface gravity g, dg_dphi (rad^-1), and dg_dh (m^-1)
            phi = deg2rad(phi_deg);
            sin_phi = sin(phi);
            cos_phi = cos(phi);
            sin_2phi = sin(2.0 * phi);

            % surface gravity (g)
            g = 9.780327 * (1.0 + 5.3024e-3 * (sin_phi^2) - 5.8e-6 * (sin_2phi^2)) ...
                - (3.0877e-6 - 4.4e-9 * (sin_phi^2)) * h ...
                + 7.2e-14 * (h^2);

            % partial derivative w.r.t latitude (phi)
            dg_dphi = 9.780327 * (1.06048e-2 * sin_phi * cos_phi - 2.32e-5 * sin_2phi * cos(2.0 * phi)) ...
                + 8.8e-9 * h * sin_phi * cos_phi;

            % partial derivative w.r.t altitude (h)
            dg_dh = -(3.0877e-6 - 4.4e-9 * (sin_phi^2)) + 1.44e-13 * h;
        end

        function wn = compute_wn(phi_deg, h, vn, ve, we, a, e2)
            % transport angular velocity in local NED frame
            radii = srekf_utils.compute_radii(phi_deg, a, e2);
            R_phi = radii(1);
            R_lamb = radii(2);

            term1 = [we * cosd(phi_deg); 0; -we * sind(phi_deg)];
            term2 = [ve / (R_lamb + h); -vn / (R_phi + h); -(ve * tand(phi_deg)) / (R_lamb + h)];
            wn = term1 + term2;
        end

        function what = compute_what(q, bias_g, sf_g, phi_deg, h, vn, ve, we, w_meas, a, e2)
            % compensates gyro measurements for bias, scale factor, and transport rate
            w_corrected = (w_meas(:) - bias_g(:)) ./ (1.0 + sf_g(:));
            wn = srekf_utils.compute_wn(phi_deg, h, vn, ve, we, a, e2);
            D_bn = srekf_utils.quaternion2dcm(q);
            D_nb = D_bn'; % Transpose is inverse for DCM
            wn_body = D_nb * wn;
            what = w_corrected - wn_body;
        end

        function ahat_n = compute_ahat(q, sf_a, bias_a, a_meas)
            % compensates accelerometer measurement and rotates body to NED frame
            a_body = (a_meas(:) - bias_a(:)) ./ (1.0 + sf_a(:));
            D_bn = srekf_utils.quaternion2dcm(q);
            ahat_n = D_bn * a_body;
        end

        function [P_baro, dP_dh] = filter_pressure(h, h_offset)
            % calculates atmospheric pressure and dP/dh at altitude h (filter_pressure.c 7th-order polynomial interpolant)
            if nargin < 2, h_offset = 35.0; end
            h_eff = h + h_offset;

            h_base = -50.0;
            h_ceil = 50000.0;
            P_0 = 100950.51;

            poly_consts = [ ...
                -0.000119272525, ...
                -1.8533997e-10, ...
                -2.4384937e-13, ...
                1.7451785e-17, ...
                -5.26659e-22, ...
                7.6701115e-27, ...
                -4.4009448e-32  ...
                ];

            if h_eff < h_base
                lognormP = 0.0059631933 + (-0.00011927191) * (h_eff - h_base);
                P_baro = P_0 * exp(lognormP);
                dP_dh = -12.04056 * exp(lognormP);
            elseif h_eff > h_ceil
                lognormP = -6.952315 + (-0.00012193789) * (h_eff - h_ceil);
                P_baro = P_0 * exp(lognormP);
                dP_dh = -12.280855 * exp(lognormP);
            else
                lognormP = 0;
                d_lognorm_dh = 0;
                for i = 1:7
                    lognormP = lognormP + poly_consts(i) * (h_eff^i);
                    d_lognorm_dh = d_lognorm_dh + i * poly_consts(i) * (h_eff^(i - 1));
                end
                P_baro = P_0 * exp(lognormP);
                dP_dh = P_baro * d_lognorm_dh;
            end
        end

    end
end


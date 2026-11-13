#!/usr/bin/env python3
"""Build the illustrated PDF report from computed Lecture 11 results."""

import json
from pathlib import Path

import pandas as pd
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Frame, Image, KeepTogether,
                                PageBreak, PageTemplate, Paragraph, Spacer, Table,
                                TableStyle)

HERE=Path(__file__).resolve().parent
RESULTS=HERE/"results"
OUT=HERE/"output"/"pdf"/"Lecture_11_Wheel_Camera_Error_Study.pdf"
OUT.parent.mkdir(parents=True,exist_ok=True)

FONTROOT=Path("/usr/share/fonts/truetype/dejavu")
pdfmetrics.registerFont(TTFont("DejaVu",str(FONTROOT/"DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DejaVu-Bold",str(FONTROOT/"DejaVuSans-Bold.ttf")))
pdfmetrics.registerFontFamily("DejaVu",normal="DejaVu",bold="DejaVu-Bold",
                              italic="DejaVu",boldItalic="DejaVu-Bold")

NAVY=colors.HexColor("#123047")
BLUE=colors.HexColor("#176B9A")
ORANGE=colors.HexColor("#DB6B38")
MINT=colors.HexColor("#2B8A78")
LIGHT=colors.HexColor("#EEF4F7")
PALE=colors.HexColor("#F8FAFB")
INK=colors.HexColor("#263747")
MUTED=colors.HexColor("#536779")

styles=getSampleStyleSheet()
styles.add(ParagraphStyle(name="TitleCustom",fontName="DejaVu-Bold",fontSize=23,
                          leading=30,textColor=NAVY,spaceAfter=13))
styles.add(ParagraphStyle(name="H1Custom",fontName="DejaVu-Bold",fontSize=16,
                          leading=21,textColor=NAVY,spaceBefore=3,spaceAfter=9))
styles.add(ParagraphStyle(name="H2Custom",fontName="DejaVu-Bold",fontSize=11,
                          leading=15,textColor=BLUE,spaceBefore=11,spaceAfter=5))
styles.add(ParagraphStyle(name="BodyCustom",fontName="DejaVu",fontSize=9.2,
                          leading=14.2,textColor=INK,spaceAfter=7))
styles.add(ParagraphStyle(name="SmallCustom",fontName="DejaVu",fontSize=7.6,
                          leading=11.2,textColor=MUTED,spaceAfter=5))
styles.add(ParagraphStyle(name="TinyCustom",fontName="DejaVu",fontSize=7.0,
                          leading=10,textColor=INK))
styles.add(ParagraphStyle(name="EquationCustom",fontName="DejaVu",fontSize=10,
                          leading=17,textColor=NAVY,spaceAfter=3))
styles.add(ParagraphStyle(name="KpiValue",fontName="DejaVu-Bold",fontSize=15,
                          leading=20,textColor=BLUE,alignment=TA_CENTER))
styles.add(ParagraphStyle(name="KpiLabel",fontName="DejaVu",fontSize=7.6,
                          leading=10,textColor=MUTED,alignment=TA_CENTER))

def P(s,style="BodyCustom"):
    return Paragraph(s,styles[style])

def section(title):
    return P(title,"H1Custom")

def subsection(title):
    return P(title,"H2Custom")

def note(text,fill=LIGHT):
    t=Table([[P(text)]],colWidths=[174*mm])
    t.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,-1),fill),
                          ("BOX",(0,0),(-1,-1),.45,colors.HexColor("#D7E5EA")),
                          ("LEFTPADDING",(0,0),(-1,-1),10),
                          ("RIGHTPADDING",(0,0),(-1,-1),10),
                          ("TOPPADDING",(0,0),(-1,-1),8),
                          ("BOTTOMPADDING",(0,0),(-1,-1),4)]))
    return t

def eq(*lines):
    t=Table([[P(s,"EquationCustom")] for s in lines],colWidths=[171*mm])
    t.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,-1),LIGHT),
                          ("LINEBEFORE",(0,0),(0,-1),3,BLUE),
                          ("LEFTPADDING",(0,0),(-1,-1),11),
                          ("RIGHTPADDING",(0,0),(-1,-1),7),
                          ("TOPPADDING",(0,0),(-1,0),7),
                          ("BOTTOMPADDING",(0,-1),(-1,-1),7)]))
    return t

def chart(name,width_mm=174):
    p=RESULTS/"figures"/name
    from PIL import Image as PILImage
    with PILImage.open(p) as image:
        w,h=image.size
    return Image(str(p),width=width_mm*mm,height=width_mm*h/w*mm)

def bullet(text):
    return P("<font color='#176B9A'>•</font>  "+text)

def stat_table(rows,widths,header=True):
    converted=[]
    for row in rows:
        converted.append([P(str(cell),"TinyCustom") for cell in row])
    t=Table(converted,colWidths=[w*mm for w in widths],repeatRows=1 if header else 0,
            hAlign="LEFT")
    rules=[("VALIGN",(0,0),(-1,-1),"MIDDLE"),
           ("LEFTPADDING",(0,0),(-1,-1),6),
           ("RIGHTPADDING",(0,0),(-1,-1),5),
           ("TOPPADDING",(0,0),(-1,-1),5),
           ("BOTTOMPADDING",(0,0),(-1,-1),5),
           ("LINEBELOW",(0,0),(-1,0),.7,BLUE)]
    if header:
        rules += [("BACKGROUND",(0,0),(-1,0),LIGHT),
                  ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.white,PALE])]
    t.setStyle(TableStyle(rules))
    return t

class Report(BaseDocTemplate):
    def __init__(self,path):
        super().__init__(str(path),pagesize=A4,leftMargin=18*mm,
                         rightMargin=18*mm,topMargin=20*mm,bottomMargin=17*mm,
                         title="Lecture 11A/11B: Wheel and Camera Error Study",
                         author="Advance Robotics experiment analysis")
        frame=Frame(self.leftMargin,self.bottomMargin,self.width,self.height,
                    leftPadding=0,rightPadding=0,topPadding=0,bottomPadding=0)
        self.addPageTemplates(PageTemplate(id="all",frames=[frame],
                                           onPage=self.decorate))
    def decorate(self,canvas,doc):
        canvas.saveState()
        if doc.page>1:
            canvas.setStrokeColor(colors.HexColor("#D9E3E8"))
            canvas.line(18*mm,282*mm,192*mm,282*mm)
            canvas.setFont("DejaVu",7.3);canvas.setFillColor(MUTED)
            canvas.drawString(18*mm,285*mm,"LECTURE 11A / 11B  •  ROBOMASTER")
        canvas.setStrokeColor(colors.HexColor("#D9E3E8"))
        canvas.line(18*mm,13*mm,192*mm,13*mm)
        canvas.setFont("DejaVu",7);canvas.setFillColor(MUTED)
        canvas.drawString(18*mm,9*mm,"Camera-based relative reference  |  13 November 2026")
        canvas.drawRightString(192*mm,9*mm,str(doc.page))
        canvas.restoreState()

def build():
    metric=pd.read_csv(RESULTS/"run_metrics.csv")
    prior=pd.read_csv(RESULTS/"phase3_paired_comparison.csv")
    summary=json.loads((RESULTS/"summary.json").read_text())
    story=[]
    add=story.append

    # Page 1: executive abstract.
    add(Spacer(1,12*mm))
    add(P("ADVANCE ROBOTICS  /  EXPERIMENT REPORT","H2Custom"))
    add(P("Wheel prediction and weighted correction", "TitleCustom"))
    add(P("Ten RoboMaster runs: wheel-only estimates, an offline Lecture 11B speed update, and the earlier Phase 3 position filter.","BodyCustom"))
    add(Spacer(1,4*mm))
    cards=[("10","complete runs"),("3.50 cm","wheel endpoint MAE"),
           ("3.51 cm","corrected endpoint MAE"),("5 / 10","runs improved")]
    cells=[[P(v,"KpiValue"),P(k,"KpiLabel")] for v,k in cards]
    table=Table([[Table([[a],[b]],colWidths=[39*mm]) for a,b in cells]],
                colWidths=[43.5*mm]*4)
    table.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,-1),LIGHT),
                              ("BOX",(0,0),(-1,-1),.6,colors.HexColor("#D3E2EA")),
                              ("INNERGRID",(0,0),(-1,-1),.5,colors.white),
                              ("TOPPADDING",(0,0),(-1,-1),13),
                              ("BOTTOMPADDING",(0,0),(-1,-1),12)]))
    add(table);add(Spacer(1,9*mm))
    add(subsection("What was measured"))
    add(P("The robot approached a box carrying AprilTag T3 ten times. Four corrected wheel angular velocities provided a forward-speed measurement. Integrating that speed produced a wheel-based position prediction. The change in camera depth to the fixed tag provided an external <b>relative</b> position reference. A 0.23 m raw ToF threshold stopped the pilot; ToF was not used as the position truth in this analysis."))
    add(subsection("Main result in ordinary words"))
    add(P("The wheels usually reported a little less travel than the camera. The mean signed difference at the finish was <b>−2.95 cm</b> (wheel minus camera; SD 2.84 cm). Camera-derived velocity is noisier because it must be calculated from changing positions; after smoothing, its paired disagreement with the wheel speed had an RMSE of <b>2.01 cm/s</b>."))
    add(note("<b>Lecture 11B result.</b> The recorded runs were replayed through a covariance-weighted wheel-speed correction. It made no meaningful improvement in endpoint position: mean absolute error 3.50 cm before and 3.51 cm after. Camera data were held back for evaluation; no robot was moved for the replay."))
    add(P("<b>Interpretation limit.</b> Errors are relative to the tag camera, not proven absolute ground-truth errors. Only the first full run had an approximate tape check of the final 20 cm front-edge clearance."))
    add(Spacer(1,5*mm))
    add(P("<b>Data:</b> full01–full10, 13 November 2026. The earlier Phase 3 comparison uses ten separate filtered trials from 6 November 2026. The attached analysis script and result tables reproduce every chart and number.","SmallCustom"))
    add(PageBreak())

    # Page 2: data flow and measurement definition.
    add(section("1  Setup and what each sensor tells us"))
    add(stat_table([
        ["Signal","Logged field","Role in this report"],
        ["Four wheel rates","omega_fr/fl/rl/rr_rad_s","Forward wheel speed; predicted travel"],
        ["AprilTag T3","t3_camera_z_m, tag_stamp_s","External relative position; smoothed velocity"],
        ["Odometry yaw","yaw_rad","Project wheel speed onto initial forward axis"],
        ["Robot odometry","x_m","Cross-check, shares wheel information"],
        ["ToF range","tof_m","Stop guard and QC only"],
        ["Requested motion","cmd_v_mps","Locate active motion and pause intervals"],
    ],[38,58,78]))
    add(Spacer(1,7*mm))
    add(subsection("Coordinate and sign convention"))
    add(P("Positive forward travel means the robot moves toward the box, making camera depth <i>z</i> smaller. Every run is reset to its own start depth so a different initial placement does not appear as a position error. The published joint-state wheel signs were already corrected by the RoboMaster driver."))
    add(eq("Camera forward displacement:  x<sub>cam</sub>(t) = median(z<sub>start</sub>) − z(t)",
           "Signed position error:  e<sub>x</sub>(t) = x<sub>wheel</sub>(t) − x<sub>cam</sub>(t)",
           "Signed velocity error:  e<sub>v</sub>(t) = v<sub>wheel,forward</sub>(t) − v<sub>cam</sub>(t)"))
    add(Spacer(1,6*mm))
    add(subsection("How the ten runs were selected"))
    add(P("Each full01–full10 file has one normal ToF stop and at least 157 moving odometry rows; every moving row has both wheel speed and camera depth. Aborted 07/08 attempts were deleted before export. In the final 07–10 runs, brief tag gaps made the pilot command zero and resume when T3 returned; the analysis masks camera gaps when forming velocity pairs."))
    add(note("<b>Why the ToF change is excluded.</b> Across these runs its start-to-end range change averages about 11 cm more than the camera displacement. The reflective target/sensor geometry was not independently calibrated over the path. It remains useful for stopping, but calling it position ground truth would change the scientific question."))
    add(PageBreak())

    # Page 3: derivation and numerical example.
    add(section("2  Lecture 11A: predict travel from wheels"))
    add(P("The four wheel rates are measured in rad/s. With nominal wheel radius <b>r = 0.050 m</b>, the forward body speed is their signed, corrected average times the radius. The robot was nearly straight; its largest observed absolute relative yaw was 0.092 rad across the runs."))
    add(eq("v<sub>wheel</sub> = (r/4)(ω<sub>FR</sub> + ω<sub>FL</sub> + ω<sub>RL</sub> + ω<sub>RR</sub>)",
           "x̂<sub>k+1</sub> = x̂<sub>k</sub> + v<sub>wheel,k</sub> cos(θ<sub>k</sub>) Δt<sub>k</sub>"))
    add(subsection("One actual logged step, from full03"))
    add(P("At approximately 5.04 s after the first forward command, the logged wheel rates were 2.30383, 2.09440, 3.03687, and 2.40855 rad/s. The next odometry timestamp was 0.11006 s later; the relative yaw was −0.02251 rad."))
    add(eq("Sum of wheel rates = 2.30383 + 2.09440 + 3.03687 + 2.40855 = 9.84365 rad/s",
           "v<sub>wheel</sub> = (0.050/4) × 9.84365 = 0.12305 m/s",
           "Δx̂ = 0.12305 × cos(−0.02251) × 0.11006 = 0.01354 m = 1.354 cm"))
    add(Spacer(1,2*mm))
    add(P("This worked step uses two adjacent logged odometry times. The reproducible script applies the same Euler equation on a uniform 0.1 s timestamp grid.","SmallCustom"))
    add(subsection("The five-state prediction behind the simple step"))
    add(P("The Phase 3 Lecture 11A model uses state [x, y, θ, v, ω] and acceleration inputs [a, α]. Its Euler update is x<sup>−</sup><sub>k+1</sub> = x<sub>k</sub> + v<sub>k</sub> cosθ<sub>k</sub> Δt; y and θ update similarly, while v and ω integrate their accelerations. The Jacobian F has F<sub>xθ</sub> = −v sinθ Δt, F<sub>xv</sub> = cosθ Δt, F<sub>yθ</sub> = v cosθ Δt, F<sub>yv</sub> = sinθ Δt, and F<sub>θω</sub> = Δt. The state uncertainty then propagates as:"))
    add(eq("P<sup>−</sup><sub>k+1</sub> = F<sub>k</sub> P<sub>k</sub> F<sub>k</sub><sup>T</sup> + G<sub>k</sub> Q G<sub>k</sub><sup>T</sup>"))
    add(note("<b>Implementation detail.</b> The original result directly integrates measured wheel speed. A separate offline replay below applies the Lecture 11B covariance update to a two-state [x,v] forward slice with yaw from odometry. The robot's original live pilot did not run this replay filter."))
    add(PageBreak())

    # Page 4: camera velocity.
    add(section("3  Camera position and velocity"))
    add(P("The AprilTag detector uses the measured <b>7.3 cm black-square width</b>, camera calibration, lens distortion, subpixel corner refinement, and solvePnP. A fixed sensor-to-tag offset cancels in the depth <i>change</i> when both sensor and box geometry remain stable. Camera depth itself is not the robot's front-edge clearance."))
    add(subsection("From depth samples to a velocity reference"))
    add(eq("x<sub>cam</sub>(t) = z<sub>0</sub> − z<sub>smooth</sub>(t)",
           "v<sub>cam</sub>(t) = − d z<sub>smooth</sub>(t) / dt"))
    add(P("Raw depth differentiation would magnify the tag's corner noise. The reproducible analysis puts the distinct camera timestamps on a 0.1 s grid, applies a three-sample median filter, then a quadratic 11-point Savitzky–Golay derivative (about 1.1 s). It excludes the first and last parts of active motion and any 0.35 s or longer camera gap plus the smoothing half-window. It also matches wheel and camera using their message timestamps instead of treating repeated last values as fresh measurements."))
    add(chart("04_velocity.png",174))
    add(P("<b>Figure.</b> full03 illustrates the paired velocity traces; the right panel shows per-run RMSE. The camera velocity is <i>derived</i> from position and is sensitive to the smoothing window.","SmallCustom"))
    add(note("<b>Sensitivity:</b> a 0.7 s window gives 2.44 cm/s velocity RMSE; the chosen 1.1 s window gives 2.01 cm/s; a 1.5 s window gives 1.82 cm/s. Longer smoothing suppresses noise but can blur acceleration and stopping."))
    add(PageBreak())

    # Page 5: numeric table.
    add(section("4  Ten-run numerical results"))
    rows=[["Run","Camera travel<br/>(m)","Wheel travel<br/>(m)",
           "Wheel − camera<br/>(cm)","Velocity RMSE<br/>(cm/s)","T3 pauses"]]
    for _,r in metric.iterrows():
        rows.append([r["run"].replace("full",""),f"{r.camera_travel_m:.3f}",
                     f"{r.wheel_travel_m:.3f}",f"{100*r.endpoint_error_m:+.2f}",
                     f"{100*r.velocity_rmse_mps:.2f}",str(int(r.pause_count))])
    add(stat_table(rows,[20,30,30,38,38,18]))
    add(Spacer(1,7*mm))
    add(subsection("Aggregate across independent runs"))
    add(stat_table([
        ["Quantity","Value","Meaning"],
        ["Camera travel, mean","1.474 m","Relative tag depth change"],
        ["Wheel travel, mean","1.445 m","Integrated signed wheel rates"],
        ["Endpoint signed error","−2.95 cm (SD 2.84)","Wheel usually reads shorter"],
        ["Endpoint absolute error","3.50 cm mean","Magnitude, ignoring sign"],
        ["Endpoint RMSE","3.99 cm","Penalizes the larger misses"],
        ["Mean-error 95% interval","−4.98 to −0.91 cm","Across ten runs, t interval"],
        ["Velocity RMSE","2.01 cm/s","1,374 valid paired time samples"],
    ],[47,50,77]))
    add(Spacer(1,6*mm))
    add(P("The mean endpoint difference is about <b>−2.0% of camera travel</b>. Full05 is the largest negative endpoint difference (−8.07 cm); full08 is positive (+2.75 cm). Raw driver odometry is close to wheel integration on average (wheel minus odometry +0.25 cm), so it is a useful consistency check, not an independent truth measurement."))
    add(note("The 95% interval describes variation between ten run-level errors under an ordinary t model. It does <b>not</b> include systematic camera scale, mounting, or tag-size uncertainty."))
    add(PageBreak())

    # Page 6: trajectories and error growth.
    add(section("5  Where position error appears"))
    add(chart("01_position_trace.png",174))
    add(P("<b>Figure.</b> In full03, the wheel integral follows the camera closely but finishes 1.84 cm shorter. The dashed cubic is the <i>nominal</i> 2.0 m/20 s plan; the pilot caps speed near the box and stops early on ToF, so the dashed curve is not a measured path or ground truth.","SmallCustom"))
    add(Spacer(1,2*mm))
    add(chart("02_position_errors.png",174))
    add(P("<b>Figure.</b> Left: signed wheel-minus-camera error over camera displacement. Right: one endpoint per complete trial; ten endpoints, rather than every time sample, are the independent repeats.","SmallCustom"))
    add(PageBreak())

    # Page 7: scatter and covariance.
    add(section("6  Lecture 11B: error scatter and covariance"))
    add(chart("03_error_scatter.png",145))
    add(P("Each dot pairs velocity and position errors at one valid moving time. Colors identify runs. The within-run centered sample covariance is <b>−2.14 × 10<sup>−5</sup> m²/s</b>, with correlation <b>r = −0.060</b>. The near-zero linear association is not strong evidence that velocity and position are independent; both errors use the same camera signal, and successive time samples are correlated.","BodyCustom"))
    add(eq("Sample covariance = Σ[(e<sub>v,i</sub> − mean e<sub>v</sub>)(e<sub>x,i</sub> − mean e<sub>x</sub>)] / (N − 1)",
           "Correlation = covariance / (SD(e<sub>v</sub>) × SD(e<sub>x</sub>))"))
    add(Spacer(1,2*mm))
    add(P("We subtract each run's mean before pooling covariance, so a run's fixed offset does not masquerade as a time-varying relationship. Seven-, eleven-, and fifteen-point smoothers all yield a small negative covariance (−2.44 to −2.13 × 10<sup>−5</sup> m²/s)."))
    add(note("<b>Do not substitute this scatter covariance directly for P<sub>xv</sub> in a Kalman gain.</b> P<sub>xv</sub> is covariance of state-prediction errors, whereas these dots compare a wheel integral and a camera-derived velocity against the same camera reference. Additional calibrated, independent noise modeling would be needed."))
    add(PageBreak())

    # Page 8: correction mathematics.
    add(section("7  Lecture 11B weighted speed correction"))
    add(P("For state [x, y, θ, v, ω], wheel speed directly observes <b>v</b> (and wheel combinations can observe ω). It does not directly measure x. This is why a speed measurement can change the position estimate instantly only when the predicted state contains cross-covariance between x and v."))
    add(eq("z = [v<sub>wheel</sub>, ω<sub>wheel</sub>]<sup>T</sup> ;   h(x) = [v, ω]<sup>T</sup>",
           "H = [[0,0,0,1,0], [0,0,0,0,1]]",
           "innovation e = z − h(x<sup>−</sup>);   S = H P<sup>−</sup> H<sup>T</sup> + R",
           "K = P<sup>−</sup> H<sup>T</sup> S<sup>−1</sup>;   x<sup>+</sup> = x<sup>−</sup> + K e"))
    add(subsection("Scalar forward-speed update: see the cross-covariance"))
    add(eq("K<sub>v</sub> = P<sub>vv</sub> / (P<sub>vv</sub> + R<sub>vv</sub>)",
           "K<sub>x</sub> = P<sub>xv</sub> / (P<sub>vv</sub> + R<sub>vv</sub>)",
           "v<sup>+</sup> = v<sup>−</sup> + K<sub>v</sub> e;   x<sup>+</sup> = x<sup>−</sup> + K<sub>x</sub> e"))
    add(P("If P<sub>xv</sub> = 0, then K<sub>x</sub> = 0 and the velocity observation does not change x immediately. Its effect can still reach future positions through the next motion prediction. We applied this update offline to all ten recorded runs; the pilot itself did not apply it live."))
    add(subsection("Why the old correction is a different experiment"))
    add(P("The earlier Phase 3 <i>adaptive_filter.py</i> uses the same five-state prediction, but its actual correction is a <b>position observation</b> H = [1,0,0,0,0]. Its filtered pilot fuses calibrated odometry progress and ToF-derived progress. That historical comparison tests an earlier position-correction system; it does not prove that the wheel-speed update above would improve these ten new runs."))
    add(PageBreak())

    # Page 9: actual update and a numerical worked example.
    add(section("8  Applying the weight to recorded runs"))
    add(P("We replay each run using state [x, v]. Between distinct wheel timestamps, x increases by v cosθ Δt and v changes by the logged command-speed difference. F = [[1, cosθ Δt], [0, 1]], G = [[0], [Δt]], and P<sup>−</sup> = F P F<sup>T</sup> + G Q G<sup>T</sup>. A wheel-speed reading z then updates both x and v using the gains on the previous page. We update covariance in Joseph form to keep it symmetric and nonnegative."))
    add(subsection("Where the weights came from"))
    add(P("For each test run, we estimate the wheel variance R and acceleration-process variance Q using only the <b>other nine runs</b>. Stationary wheel noise gives a robust wheel standard deviation of 0.00776 m/s. Command-minus-wheel velocity increments give an acceleration standard deviation of 0.240–0.263 m/s². Each replay starts with 2 cm position SD and 5 cm/s speed SD. These are practical proxies, not independently calibrated motion-noise measurements. No tag depth enters the tuning or correction."))
    add(eq("R = σ²<sub>wheel</sub>;  Q = σ²<sub>acceleration</sub>",
           "K<sub>v</sub> = P<sup>−</sup><sub>vv</sub> / (P<sup>−</sup><sub>vv</sub> + R)",
           "K<sub>x</sub> = P<sup>−</sup><sub>xv</sub> / (P<sup>−</sup><sub>vv</sub> + R)"))
    add(subsection("One actual update, full03 near 5 seconds"))
    add(P("The wheel reports 0.123046 m/s and the prediction is about 0.119378 m/s, leaving innovation <b>+0.003668 m/s</b>. At that moment K<sub>v</sub> = 0.943 and K<sub>x</sub> = 0.00681 s. Thus the speed changes by about <b>+0.00346 m/s</b> and position by only <b>+0.000025 m</b> (0.025 mm). The reported mean gains over all runs are 0.925 for speed and 0.00789 s for position."))
    add(chart("08_kalman_weights.png",151))
    add(P("<b>Figure.</b> The speed gain is high; the direct position change from each speed reading is small. K<sub>x</sub> has units of seconds and should not be interpreted as a percentage.","SmallCustom"))
    add(PageBreak())

    # Page 10: new ten-run correction comparison.
    add(section("9  Before and after: the same ten runs"))
    add(stat_table([
        ["Measure","Wheel only","Weighted correction"],
        ["Endpoint mean absolute error","3.495 cm","3.506 cm"],
        ["Endpoint RMSE","3.992 cm","3.965 cm"],
        ["Mean signed endpoint error","−2.946 cm","−3.012 cm"],
        ["Velocity RMSE against camera","2.006 cm/s","1.919 cm/s"],
        ["Trajectory RMSE, mean per run","3.019 cm","3.060 cm"],
    ],[78,48,48]))
    add(P("The corrected endpoint absolute error is lower in <b>5 of 10</b> runs. Its mean change is <b>0.011 cm worse</b>; the paired 95% interval for improvement spans −0.229 to +0.207 cm. The small RMSE changes disagree in direction, so this sample does not demonstrate a position benefit. The slight velocity RMSE reduction is a comparison to <i>camera-derived, smoothed</i> speed and should be read with its smoothing limitation."))
    add(chart("06_weighted_correction_endpoints.png",160))
    add(P("<b>Figure.</b> Each pair shows absolute endpoint error against the same camera travel. Some runs improve and others worsen. Using identical wheel timestamps for a separate matched wheel-only integration gives 3.483 cm MAE, also slightly below the 3.506 cm correction. This checks that a changed numerical time grid is not mistaken for a filter gain.","SmallCustom"))
    add(P("<b>Why little changes:</b> the correction uses the <i>same wheels</i> that supply the wheel-only trajectory, so it adds no independent observation of position. It can temper instantaneous speed noise but cannot identify a systematic wheel scale bias. This is a two-state forward slice of the five-state model, applied offline; it is not a live navigation result.","SmallCustom"))
    add(PageBreak())

    # Page 11: paired error trajectory for every completed run.
    add(section("10  Position error through time, before and after"))
    add(P("Each panel shows one run. At every valid camera time, we subtract the <b>same camera displacement</b> from the wheel-only estimate and the corrected estimate. The horizontal dashed line is zero error. Below it, the estimate says the robot traveled less than the camera says."))
    add(chart("09_error_trajectories_all_runs.png",158))
    add(P("<b>Figure.</b> Orange dashed: wheel-only error; blue solid: corrected error. Gaps in valid tag data are not connected. The paths nearly overlap in most runs, showing that the weighted speed update made only small changes. These traces use smoothed camera depth at each instant; the endpoint table uses stationary before/after camera medians, so their last plotted values need not match the endpoint numbers exactly.","SmallCustom"))
    add(PageBreak())

    # Page 12: retrospective correction.
    add(section("11  Earlier Phase 3 correction comparison"))
    add(P("A retrospective paired comparison uses the ten older <i>filtered_01–10</i> recorder files and their matching <i>filtered_state</i> logs. Each endpoint is compared with that trial's camera depth change. The old detector assumed an 8.0 cm tag; its depth change is scaled by the subsequently measured 7.3/8.0 size ratio. This is an approximate retrospective calibration, not a new physical ground-truth survey."))
    add(chart("05_phase3_comparison.png",174))
    add(P("<b>Earlier ten paired trials:</b> raw odometry mean absolute error 5.25 cm; earlier filter 3.96 cm. Mean paired reduction is <b>1.29 cm</b> (about 24%); 9/10 individual trials improve. A simple run-level 95% t interval for the mean paired reduction is 0.63–1.94 cm. Run 01 worsens slightly. The comparison is descriptive because the old filter was calibrated using separate earlier unfiltered runs, the tag was small at the far start, and its camera reference has uncertainty.","BodyCustom"))
    add(note("<b>Keep the claims separate.</b> Our new ten-run weighted speed update has no demonstrated endpoint benefit. The old ten filtered runs use direct position information and improved relative to raw odometry in 9/10 trials. They are different sensors, runs, and correction designs."))
    add(PageBreak())

    # Page 13: implementation and limits.
    add(section("12  Code map, verification, and next step"))
    add(stat_table([
        ["File in analysis package or source archive","What it implements"],
        ["lecture11_wheel_error/code/record_wheel_camera.py","Time-stamped camera, ToF, four wheels, odometry, commands"],
        ["lecture11_wheel_error/code/tag_pose.py","Measured tag width; subpixel AprilTag pose"],
        ["lecture11_wheel_error/code/pilot_20cm.py","Camera target guard, ToF stop, brief tag-loss pause/resume"],
        ["code/rm_phase3_estimation/.../motion_model.py","Five-state Euler prediction, F and G Jacobians"],
        ["code/rm_phase3_estimation/.../measurement.py","Lecture 11B speed measurement H and scalar gain"],
        ["code/rm_phase3_estimation/.../adaptive_filter.py","Earlier Phase 3 position EKF and Joseph covariance update"],
        ["lecture11_analysis.py in analysis package","Offline time alignment, integration, velocity smoothing, metrics, figures"],
        ["lecture11_correction.py in analysis package","Offline 11B two-state weighted update, held-out calibration, comparisons"],
    ],[79,95]))
    add(Spacer(1,5*mm))
    add(subsection("What was verified"))
    add(bullet(f"Ten normal pilot stops, ten unique CSVs, and complete camera/wheel values for all {int(metric.moving_rows.sum()):,} moving odometry rows."))
    add(bullet("Message timestamps, distinct tag and wheel samples, camera-gap masks, and stationary start/end medians were used."))
    add(bullet("The first full trial ended at approximately 20 cm front-edge clearance by tape; other endpoints were not independently taped."))
    add(subsection("Limitations and a defensible next experiment"))
    add(P("The 7.3 cm tag's camera depth can have corner and scale bias; a stationary refined run had about 14.7 mm depth SD. Motion blur, tag loss, unequal resetting, yaw, and the assumed 0.050 m wheel radius can also affect the comparison. A longer visible tag or a measured rail/tape position at several points would test the camera as reference before identifying a wheel-radius correction."))
    add(P("We did apply the weighted update in a leave-one-run-out replay. For a stronger claim, independently calibrate wheel noise and process motion noise under motion, measure position at several points with a larger well-calibrated target or tape/rail, and then test the frozen filter on fresh runs. The current stationary-noise R proxy may not describe wheel error while moving."))
    add(Spacer(1,3*mm))
    add(P("<b>Reproduce:</b> extract the input ZIP, then run <font face='DejaVu'>python lecture11_analysis.py --input-dir EXTRACTED_ROOT --output-dir results</font>, followed by <font face='DejaVu'>python lecture11_correction.py --input-dir EXTRACTED_ROOT --baseline-results results --output-dir results</font>. Dependencies: numpy, pandas, scipy, matplotlib; ReportLab and Pillow for the PDF.","SmallCustom"))
    add(PageBreak())

    # Page 14: compact formulas and source provenance.
    add(section("Appendix  Quick revision sheet"))
    add(subsection("Prediction, observation, error"))
    add(eq("1. Four wheel rates → v<sub>wheel</sub> = r Σω<sub>i</sub> / 4",
           "2. Predict x̂<sub>k+1</sub> = x̂<sub>k</sub> + v<sub>wheel,k</sub> cosθ<sub>k</sub> Δt<sub>k</sub>",
           "3. Camera travel x<sub>cam</sub> = z<sub>start</sub> − z<sub>current</sub>",
           "4. Position error e<sub>x</sub> = x̂ − x<sub>cam</sub>",
           "5. Camera speed v<sub>cam</sub> = −d z<sub>smooth</sub>/dt; speed error e<sub>v</sub> = v<sub>wheel</sub> − v<sub>cam</sub>"))
    add(subsection("Correction applied offline here"))
    add(eq("6. e = z − h(x<sup>−</sup>) ;  S = H P<sup>−</sup> H<sup>T</sup> + R",
           "7. K = P<sup>−</sup> H<sup>T</sup> S<sup>−1</sup> ;  x<sup>+</sup> = x<sup>−</sup> + K e",
           "8. Wheel speed observes v, not x; P<sub>xv</sub> controls immediate x correction"))
    add(subsection("Reading the sign"))
    add(P("A negative endpoint error means the wheel integral says the robot moved <b>less</b> than the camera says. A positive error means it says the robot moved <b>more</b>. An RMSE describes typical magnitude and cannot show which direction dominates. Covariance units are m²/s, since it multiplies a position error (m) by a velocity error (m/s)."))
    add(subsection("Audit trail"))
    add(P("Source: user-supplied <i>lecture11_analysis_input.zip</i> containing the ten completed experiment CSVs, pilot logs, stationary tests, and earlier Phase 3 code/data. Results were computed offline from that snapshot. Aborted retries were absent from the supplied archive. No new robot motion was commanded during this analysis."))

    Report(OUT).build(story)
    print(OUT)

if __name__=="__main__":
    build()

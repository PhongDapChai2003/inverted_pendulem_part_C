from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    KeepTogether,
    PageTemplate,
    Paragraph,
    PageBreak,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output" / "pdf" / "track_c_advanced_swing_up_implementation_plan.pdf"
OUTPUT.parent.mkdir(parents=True, exist_ok=True)

NAVY = colors.HexColor("#17324D")
BLUE = colors.HexColor("#2A6F97")
LIGHT = colors.HexColor("#EAF3F8")
GOLD = colors.HexColor("#D99A2B")
RED = colors.HexColor("#A33A3A")
INK = colors.HexColor("#1D2730")
MUTED = colors.HexColor("#536270")

styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name="TitleX", parent=styles["Title"], fontName="Helvetica-Bold", fontSize=22, leading=26, textColor=NAVY, alignment=TA_CENTER, spaceAfter=14))
styles.add(ParagraphStyle(name="SubtitleX", parent=styles["Normal"], fontName="Helvetica", fontSize=11, leading=16, textColor=MUTED, alignment=TA_CENTER, spaceAfter=12))
styles.add(ParagraphStyle(name="H1X", parent=styles["Heading1"], fontName="Helvetica-Bold", fontSize=15, leading=18, textColor=NAVY, spaceBefore=12, spaceAfter=7))
styles.add(ParagraphStyle(name="H2X", parent=styles["Heading2"], fontName="Helvetica-Bold", fontSize=11.5, leading=14, textColor=BLUE, spaceBefore=8, spaceAfter=4))
styles.add(ParagraphStyle(name="BodyX", parent=styles["BodyText"], fontName="Helvetica", fontSize=9, leading=12.4, textColor=INK, spaceAfter=5))
styles.add(ParagraphStyle(name="BulletX", parent=styles["BodyText"], fontName="Helvetica", fontSize=8.8, leading=12, leftIndent=14, firstLineIndent=-7, bulletIndent=5, textColor=INK, spaceAfter=3))
styles.add(ParagraphStyle(name="CodeX", parent=styles["Code"], fontName="Courier", fontSize=7.3, leading=10, leftIndent=8, rightIndent=8, borderColor=colors.HexColor("#B8CAD6"), borderWidth=0.6, borderPadding=7, backColor=colors.HexColor("#F4F8FA"), textColor=INK, spaceBefore=4, spaceAfter=7))
styles.add(ParagraphStyle(name="CalloutX", parent=styles["BodyText"], fontName="Helvetica-Bold", fontSize=9, leading=12.5, leftIndent=8, rightIndent=8, borderColor=GOLD, borderWidth=1, borderPadding=8, backColor=colors.HexColor("#FFF8E7"), textColor=NAVY, spaceBefore=6, spaceAfter=8))
styles.add(ParagraphStyle(name="SmallX", parent=styles["BodyText"], fontName="Helvetica", fontSize=7.5, leading=10, textColor=MUTED, spaceAfter=3))


def p(text, style="BodyX"):
    return Paragraph(text, styles[style])


def bullets(items):
    return [Paragraph("• " + escape(item), styles["BulletX"]) for item in items]


def header_footer(canvas, doc):
    canvas.saveState()
    width, height = letter
    canvas.setStrokeColor(colors.HexColor("#B8CAD6"))
    canvas.setLineWidth(0.5)
    canvas.line(0.65 * inch, height - 0.48 * inch, width - 0.65 * inch, height - 0.48 * inch)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(MUTED)
    canvas.drawString(0.65 * inch, height - 0.38 * inch, "TRACK C | ADVANCED SWING-UP")
    canvas.drawRightString(width - 0.65 * inch, 0.38 * inch, f"Page {doc.page}")
    canvas.restoreState()


doc = BaseDocTemplate(
    str(OUTPUT), pagesize=letter,
    leftMargin=0.67 * inch, rightMargin=0.67 * inch,
    topMargin=0.65 * inch, bottomMargin=0.58 * inch,
    title="Track C Advanced Swing-Up: Corrected Implementation Plan",
    author="Pendulum Project Team",
)
frame = Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height, id="main")
doc.addPageTemplates([PageTemplate(id="main", frames=[frame], onPage=header_footer)])

story = []
story += [Spacer(1, 0.35 * inch), p("Track C Advanced Swing-Up", "TitleX"), p("Corrected implementation plan for an energy-based swing-up and LQR-stabilized linear cart-pole", "SubtitleX")]
summary = Table([
    [p("CONTROL", "SmallX"), p("PLANT", "SmallX"), p("PRIMARY GATE", "SmallX")],
    [p("Energy shaping + discrete LQR"), p("Linear cart-pole"), p("18 successful autonomous trials out of 20")],
], colWidths=[2.1*inch, 1.65*inch, 2.25*inch])
summary.setStyle(TableStyle([
    ("BACKGROUND", (0,0), (-1,0), NAVY), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
    ("BACKGROUND", (0,1), (-1,1), LIGHT), ("BOX", (0,0), (-1,-1), 0.8, BLUE),
    ("INNERGRID", (0,0), (-1,-1), 0.4, colors.HexColor("#A8BECF")),
    ("VALIGN", (0,0), (-1,-1), "MIDDLE"), ("LEFTPADDING", (0,0), (-1,-1), 7),
    ("RIGHTPADDING", (0,0), (-1,-1), 7), ("TOPPADDING", (0,0), (-1,-1), 6), ("BOTTOMPADDING", (0,0), (-1,-1), 6),
]))
story += [summary, Spacer(1, 0.18*inch)]
story += [p("Decision", "H1X"), p("Proceed with the architecture, but do not build directly from the original two-page proposal. It is conceptually correct and operationally incomplete. The project needs fixed conventions, an identified plant, a motor inner loop, capture hysteresis, rail/current/timing protection, and numerical exit gates.", "CalloutX")]

story += [p("What the references establish", "H1X")]
refs = [
    ("Linear swing-up demonstration", "Scott Rumschlag's video validates the physical concept and highlights motor performance/feedback plus continuous speed and acceleration monitoring."),
    ("Rotary PID project", "Cillk 11's plant is rotary, not linear. Its Arduino Mega, geared motor, two encoders, and L298N are evidence of one build, not a recommended bill of materials. The presentation lists LQR and energy forcing as future work."),
    ("Nonlinear model", "Brianno Coller's Newtonian derivation is the relevant foundation for the linear cart-pole. Use the nonlinear model for swing-up and linearize only near upright."),
]
for name, body in refs:
    story.append(KeepTogether([p(name, "H2X"), p(body)]))

story += [p("Corrections to the proposal", "H1X")]
corrections = [
    [p("Proposal item", "SmallX"), p("Corrected requirement", "SmallX")],
    [p("Arduino Uno or STM32"), p("Prefer STM32-class timing and diagnostics margin; use Uno only for a deliberately reduced prototype.")],
    [p("Ultrasonic sensor"), p("Use only as a coarse sanity check. Cart position must come from a deterministic encoder.")],
    [p("Switch near upright"), p("Gate on angle, angular velocity, cart position, dwell time, and safety state; add hysteresis and command blending.")],
    [p("Rise time and overshoot"), p("Confirm official definitions. A wrapped 180-degree maneuver makes generic 10%-90% angle rise time ambiguous.")],
]
t = Table(corrections, colWidths=[1.65*inch, 4.35*inch], repeatRows=1)
t.setStyle(TableStyle([
    ("BACKGROUND", (0,0), (-1,0), NAVY), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
    ("GRID", (0,0), (-1,-1), 0.45, colors.HexColor("#A8BECF")), ("VALIGN", (0,0), (-1,-1), "TOP"),
    ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, LIGHT]),
    ("LEFTPADDING", (0,0), (-1,-1), 6), ("RIGHTPADDING", (0,0), (-1,-1), 6),
    ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
]))
story += [t, PageBreak()]

story += [p("1. Model and conventions", "H1X")]
story += bullets([
    "x = 0 at rail center; positive in the calibrated motor direction.",
    "theta = 0 upright and theta = pi hanging downward; wrap error to [-pi, pi).",
    "State z = [x, x_dot, theta, theta_dot]^T; use SI units everywhere.",
    "Controller output is requested force or acceleration; a calibrated inner motor loop produces PWM/current.",
])
story += [p("Use cart mass M, pendulum mass m, pivot-to-COM distance l, COM inertia I, cart friction b, pivot damping c, gravity g, and actuator force F. Define J = I + m l^2.")]
story += [p("(M + m) x_ddot + b x_dot + m l (theta_ddot cos(theta) - theta_dot^2 sin(theta)) = F<br/>J theta_ddot + c theta_dot - m g l sin(theta) + m l x_ddot cos(theta) = 0", "CodeX")]
story += [p("Derive the upright linear model from these same equations and measured parameters. Copying gains from a different plant is not valid.", "CalloutX")]

story += [p("2. Hardware baseline", "H1X")]
story += bullets([
    "Rigid low-play rail and belt transmission with enough usable travel for energy pumping.",
    "Bidirectional motor/driver with controlled braking, current sensing, and measured acceleration headroom.",
    "Quadrature pendulum encoder plus slip-free cart-position encoder; direct linear measurement is preferred.",
    "STM32-class controller, fused supply, emergency stop, independent bridge enable, normally closed limit switches, and energy-absorbing mechanical stops.",
    "Modern low-loss H-bridge sized from stall and transient current. L298N is not the default because its voltage drop and thermal loss work against a fast cart.",
])

story += [p("3. Real-time architecture", "H1X")]
story += bullets([
    "Start with a 1 kHz control interrupt; accept 500 Hz only after timing and stability measurements.",
    "Read hardware encoder counters atomically; filter finite-difference velocities with a measured low-lag IIR filter.",
    "Run safety checks before control calculation and before applying PWM.",
    "Apply command saturation, slew-rate limiting, time-current limiting, and a watchdog.",
    "Log time, mode, all four states, command, PWM, current, voltage, and fault flags.",
])

story += [p("4. State machine", "H1X"), p("BOOT -> CALIBRATE -> IDLE -> SWING_UP -> CAPTURE -> BALANCE; any active state can enter latched FAULT.", "CodeX")]
state_rows = [
    [p("Mode", "SmallX"), p("Responsibility", "SmallX")],
    [p("CALIBRATE"), p("Verify encoder directions, center the rail slowly, test limits/current sensing, and require deliberate arm.")],
    [p("SWING_UP"), p("Shape pendulum energy while centering the cart and enforcing rail/actuator limits.")],
    [p("CAPTURE"), p("Blend from swing-up to LQR over 50-150 ms so motor command does not step.")],
    [p("BALANCE"), p("Discrete full-state LQR with optional slow integral correction on cart position.")],
    [p("FAULT"), p("Disable the bridge, preserve telemetry, and require explicit reset.")],
]
t2 = Table(state_rows, colWidths=[1.25*inch, 4.75*inch], repeatRows=1)
t2.setStyle(TableStyle([
    ("BACKGROUND", (0,0), (-1,0), NAVY), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
    ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor("#A8BECF")), ("VALIGN", (0,0), (-1,-1), "TOP"),
    ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, LIGHT]),
    ("LEFTPADDING", (0,0), (-1,-1), 6), ("RIGHTPADDING", (0,0), (-1,-1), 6),
    ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
]))
story += [t2, PageBreak()]

story += [p("5. Swing-up and capture", "H1X")]
story += [p("Energy relative to upright and a starting energy-shaping law:")]
story += [p("E = 0.5 J theta_dot^2 + m g l (cos(theta) - 1); E_target = 0<br/>a_energy = k_E E theta_dot cos(theta)<br/>a_center = -k_x_su x - k_v_su x_dot<br/>a_cmd = saturate(a_energy + a_center, -a_max, +a_max)", "CodeX")]
story += [p("Determine final motor polarity with a low-power calibration. Smooth the zero-speed sign change, reduce pumping in the soft-limit zone, and dump energy/recenter if a safe catch is no longer possible.")]

story += [p("Discrete LQR", "H2X")]
story += [p("Linearize the identified model about upright, discretize at the measured sample period, and compute u = -Kz. Normalize Q and R by physical state and command limits so weights remain interpretable.")]
story += [p("Q = diag(1/x_allow^2, 1/v_allow^2, 1/theta_allow^2, 1/omega_allow^2)<br/>R = 1/u_allow^2", "CodeX")]

story += [p("Initial transition gates", "H2X")]
gate_data = [
    [p("Transition", "SmallX"), p("Starting condition", "SmallX")],
    [p("SWING_UP -> CAPTURE"), p("|theta| < 12 deg, |theta_dot| < 1.5 rad/s, |x| < 0.60 x_max for 30 ms")],
    [p("CAPTURE -> BALANCE"), p("|theta| < 8 deg continuously for 100 ms")],
    [p("BALANCE -> SWING_UP"), p("|theta| > 20 deg or |theta_dot| > 2.5 rad/s, only while rail state remains safe")],
    [p("Any -> FAULT"), p("Hard/soft limit trip, overcurrent, invalid encoder, watchdog, or repeated timing overrun")],
]
t3 = Table(gate_data, colWidths=[1.7*inch, 4.3*inch], repeatRows=1)
t3.setStyle(TableStyle([
    ("BACKGROUND", (0,0), (-1,0), NAVY), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
    ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor("#A8BECF")), ("VALIGN", (0,0), (-1,-1), "TOP"),
    ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, LIGHT]),
    ("LEFTPADDING", (0,0), (-1,-1), 6), ("RIGHTPADDING", (0,0), (-1,-1), 6),
    ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
]))
story += [t3, p("These thresholds are commissioning values, not final claims. Hysteresis and dwell time are mandatory.", "SmallX")]

story += [p("6. Safety envelope", "H1X")]
safety = [
    [p("Zone", "SmallX"), p("Cart position", "SmallX"), p("Required behavior", "SmallX")],
    [p("Green"), p("<= 70% x_max"), p("Normal control")],
    [p("Amber"), p("70%-85%"), p("Reduce energy injection; bias toward center")],
    [p("Red"), p("85%-95%"), p("Cancel swing-up; bounded braking/centering only")],
    [p("Trip"), p("> 95% or hard limit"), p("Disable bridge and latch FAULT")],
]
t4 = Table(safety, colWidths=[1*inch, 1.35*inch, 3.65*inch], repeatRows=1)
t4.setStyle(TableStyle([
    ("BACKGROUND", (0,0), (-1,0), NAVY), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
    ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor("#A8BECF")), ("VALIGN", (0,0), (-1,-1), "TOP"),
    ("BACKGROUND", (0,1), (-1,1), colors.HexColor("#E8F4EA")),
    ("BACKGROUND", (0,2), (-1,2), colors.HexColor("#FFF4CF")),
    ("BACKGROUND", (0,3), (-1,3), colors.HexColor("#FCE5D0")),
    ("BACKGROUND", (0,4), (-1,4), colors.HexColor("#F8DADA")),
    ("LEFTPADDING", (0,0), (-1,-1), 6), ("RIGHTPADDING", (0,0), (-1,-1), 6),
    ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
]))
story += [t4, PageBreak()]

story += [p("7. Development sequence and exit gates", "H1X")]
phases = [
    ("1. Mechanical/electrical checkout", "Measure travel, masses, center of mass, inertia estimate, backlash, current, and supply behavior. Exit when both limits and emergency stop reliably stop motion and all encoder signs are documented."),
    ("2. Motor characterization", "Map command/current to acceleration in both directions; measure deadband, braking, friction, saturation, and delay. Exit with a safe acceleration and time-current envelope."),
    ("3. State estimation", "Calibrate counts per meter/radian and filters. Exit with no missed counts in worst-case motion and quantified velocity noise/lag."),
    ("4. Identification and simulation", "Fit M, m, l, I, b, and c. Exit when free-swing and cart-step direction, frequency, and damping match within a documented tolerance."),
    ("5. Upright stabilization first", "Manually place near upright with swing-up disabled. Exit after at least 20 successful low-energy catches with acceptable centering."),
    ("6. Swing-up at reduced power", "Tune energy gain and centering without capture. Exit when the system reaches the capture region repeatably without leaving the amber zone."),
    ("7. Capture integration", "Enable dwell, hysteresis, and blending. Exit after at least 18 successful autonomous catches in 20 consecutive trials."),
    ("8. Optimize", "Tune one factor at a time only after repeatability. Store controller version and parameters with every log."),
]
for title, body in phases:
    story.append(KeepTogether([p(title, "H2X"), p(body)]))

story += [p("8. Verification matrix", "H1X")]
story += bullets([
    "Encoder direction, wraparound, disconnect, and implausible-jump tests.",
    "Actuator symmetry, braking, deadband, saturation, current limit, and supply sag.",
    "Hard limits, software zones, emergency stop, watchdog, overcurrent, and timing overrun.",
    "Catch map across initial angle/angular-velocity combinations.",
    "Swing-up repeatability, disturbance recovery, reduced travel, and thermal soak.",
    "Simulation-versus-hardware overlays for angle, cart position, and motor command.",
])

story += [p("9. Metrics and acceptance", "H1X")]
story += [p("The generic 10%-90% rise-time definition in the proposal is ambiguous for a wrapped 180-degree maneuver. Obtain the official Track C rule text before optimizing and encode its exact start event, target, settling band, timeout, and failure handling.", "CalloutX")]
story += bullets([
    "Report swing-up time, capture time, total time to one second stable balance, peak post-capture angle, settling time, RMS errors, peak current, energy, and minimum rail clearance.",
    "Count every safety fault as a failed trial; do not discard inconvenient runs.",
    "Provisional acceptance: at least 18/20 autonomous successes, 30 seconds of balance, no hard-stop contact, no sustained overcurrent, no missed deadlines, and a complete raw log per trial.",
])

story += [p("Immediate decisions", "H1X")]
story += bullets([
    "Confirm the official rules and physical/power constraints.",
    "Choose usable rail travel and pendulum/cart geometry.",
    "Size motor, transmission, driver, and supply from required peak acceleration and measured current.",
    "Choose encoder resolution for the velocity-noise target at the control rate.",
    "Freeze the sign convention and safety wiring before controller implementation.",
])

story += [p("Definition of done", "H1X"), p("A cold-start, hands-free run calibrates, swings up, captures, and balances for 30 seconds; respects rail, current, and timing envelopes; produces a complete log; and succeeds in at least 18 of 20 consecutive trials.", "CalloutX")]

story += [Spacer(1, 0.12*inch), p("Sources", "H1X")]
story += [p("Original proposal: Proposed Pland Track C: Advanced Swing Up (2 pages).", "SmallX")]
for url in [
    "https://www.youtube.com/watch?v=hQK_3C6S4Ak",
    "https://www.youtube.com/watch?v=bY4t6yfBA24",
    "https://www.youtube.com/watch?v=5qJY-ZaKSic",
]:
    story.append(p(f'<link href="{url}" color="#2A6F97">{url}</link>', "SmallX"))

doc.build(story)
print(OUTPUT)

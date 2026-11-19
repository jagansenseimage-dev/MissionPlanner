using log4net;
using MissionPlanner.ArduPilot;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Windows.Forms;

namespace MissionPlanner.Controls
{
    /// <summary>
    /// Actions tab flight-mode buttons, split into four vehicle categories
    /// (Multicopter, VTOL / QuadPlane, Fixed-Wing Plane, Rover).
    ///
    /// Each button carries the ArduPilot custom_mode number for its vehicle type
    /// (taken from ArduCopter/mode.h, ArduPlane/mode.h and Rover/mode.h). The same
    /// number means different things on different firmwares (e.g. 10 is AUTO on Plane
    /// and Rover but undefined on Copter), so a category is only enabled when the
    /// connected vehicle's firmware matches it, and this is re-checked on every click
    /// before anything is sent.
    ///
    /// The request goes through MAVLinkInterface.setMode(sysid, compid, mavlink_set_mode_t),
    /// the same path the existing "Set Mode" button ends up in, which sends
    /// MAV_CMD_DO_SET_MODE plus the legacy SET_MODE message.
    /// </summary>
    public class FlightModeButtonPanel : UserControl
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        public enum ModeCategory
        {
            Multicopter,
            Vtol,
            FixedWing,
            Rover
        }

        private sealed class ModeDefinition
        {
            public ModeCategory Category;
            public string Label;
            public uint CustomMode;

            public ModeDefinition(ModeCategory category, string label, uint customMode)
            {
                Category = category;
                Label = label;
                CustomMode = customMode;
            }
        }

        private sealed class Section
        {
            public ModeCategory Category;
            public string Title;
            public Color Accent;
            public string Status = "";
            public bool Available;
            public Rectangle HeaderBounds;
            public readonly List<MyButton> Buttons = new List<MyButton>();
        }

        // Mode numbers verified against ArduPilot master: ArduCopter/mode.h, ArduPlane/mode.h, Rover/mode.h
        private static readonly ModeDefinition[] Modes =
        {
            // MULTICOPTER (ArduCopter)
            new ModeDefinition(ModeCategory.Multicopter, "Stabilize", 0),
            new ModeDefinition(ModeCategory.Multicopter, "Acro", 1),
            new ModeDefinition(ModeCategory.Multicopter, "Alt Hold", 2),
            new ModeDefinition(ModeCategory.Multicopter, "Auto", 3),
            new ModeDefinition(ModeCategory.Multicopter, "Guided", 4),
            new ModeDefinition(ModeCategory.Multicopter, "Loiter", 5),
            new ModeDefinition(ModeCategory.Multicopter, "RTL", 6),
            new ModeDefinition(ModeCategory.Multicopter, "Circle", 7),
            new ModeDefinition(ModeCategory.Multicopter, "Land", 9),
            new ModeDefinition(ModeCategory.Multicopter, "PosHold", 16),
            new ModeDefinition(ModeCategory.Multicopter, "Brake", 17),
            new ModeDefinition(ModeCategory.Multicopter, "Smart RTL", 21),

            // VTOL / QUADPLANE (ArduPlane with Q_ENABLE=1, VTOL modes)
            new ModeDefinition(ModeCategory.Vtol, "QStabilize", 17),
            new ModeDefinition(ModeCategory.Vtol, "QHover", 18),
            new ModeDefinition(ModeCategory.Vtol, "QLoiter", 19),
            new ModeDefinition(ModeCategory.Vtol, "QLand", 20),
            new ModeDefinition(ModeCategory.Vtol, "QRTL", 21),
            new ModeDefinition(ModeCategory.Vtol, "QAcro", 23),
            new ModeDefinition(ModeCategory.Vtol, "QAutotune", 22),
            new ModeDefinition(ModeCategory.Vtol, "Loiter-QLand", 25),

            // FIXED-WING PLANE (ArduPlane)
            new ModeDefinition(ModeCategory.FixedWing, "Manual", 0),
            new ModeDefinition(ModeCategory.FixedWing, "Stabilize", 2),
            new ModeDefinition(ModeCategory.FixedWing, "FBWA", 5),
            new ModeDefinition(ModeCategory.FixedWing, "FBWB", 6),
            new ModeDefinition(ModeCategory.FixedWing, "Cruise", 7),
            new ModeDefinition(ModeCategory.FixedWing, "Auto", 10),
            new ModeDefinition(ModeCategory.FixedWing, "RTL", 11),
            new ModeDefinition(ModeCategory.FixedWing, "Loiter", 12),
            new ModeDefinition(ModeCategory.FixedWing, "Circle", 1),
            new ModeDefinition(ModeCategory.FixedWing, "Guided", 15),
            new ModeDefinition(ModeCategory.FixedWing, "Takeoff", 13),
            new ModeDefinition(ModeCategory.FixedWing, "Acro", 4),

            // ROVER (ArduRover / boats)
            new ModeDefinition(ModeCategory.Rover, "Manual", 0),
            new ModeDefinition(ModeCategory.Rover, "Acro", 1),
            new ModeDefinition(ModeCategory.Rover, "Steering", 3),
            new ModeDefinition(ModeCategory.Rover, "Hold", 4),
            new ModeDefinition(ModeCategory.Rover, "Loiter", 5),
            new ModeDefinition(ModeCategory.Rover, "Auto", 10),
            new ModeDefinition(ModeCategory.Rover, "Guided", 15),
            new ModeDefinition(ModeCategory.Rover, "RTL", 11),
            new ModeDefinition(ModeCategory.Rover, "Smart RTL", 12),
            new ModeDefinition(ModeCategory.Rover, "Follow", 6),
        };

        private static readonly Color ActiveTop = Color.FromArgb(0x4C, 0xC4, 0x4C);
        private static readonly Color ActiveBottom = Color.FromArgb(0x1F, 0x8A, 0x1F);

        private const int SectionGap = 6;
        private const int ButtonGap = 3;
        private const int OuterPad = 3;
        private const int AccentWidth = 4;
        private const int ConfirmTimeoutMs = 3000;

        private readonly List<Section> sections = new List<Section>();
        private readonly Dictionary<MyButton, Color[]> savedColors = new Dictionary<MyButton, Color[]>();
        private readonly Timer stateTimer = new Timer();
        private Font headerFont;
        private Font statusFont;

        private string cachedModeText;
        private Firmwares cachedFirmware;
        private int cachedModeNumber = -1;

        private ModeDefinition pendingMode;
        private DateTime pendingSince;
        private bool layingOut;

        public FlightModeButtonPanel()
        {
            DoubleBuffered = true;
            SetStyle(ControlStyles.ResizeRedraw, true);

            CreateFonts();

            AddSection(ModeCategory.Multicopter, "MULTICOPTER", Color.FromArgb(0x2E, 0x86, 0xDE));
            AddSection(ModeCategory.Vtol, "VTOL / QUADPLANE", Color.FromArgb(0x8E, 0x44, 0xAD));
            AddSection(ModeCategory.FixedWing, "FIXED-WING PLANE", Color.FromArgb(0xE6, 0x7E, 0x22));
            AddSection(ModeCategory.Rover, "ROVER", Color.FromArgb(0x27, 0xAE, 0x60));

            stateTimer.Interval = 500;
            stateTimer.Tick += (s, e) => UpdateState();
            stateTimer.Start();

            UpdateState();
            DoLayout();
        }

        private void AddSection(ModeCategory category, string title, Color accent)
        {
            var section = new Section { Category = category, Title = title, Accent = accent };

            foreach (var mode in Modes.Where(m => m.Category == category))
            {
                var but = new MyButton
                {
                    Text = mode.Label,
                    Tag = mode,
                    Name = "BUT_mode_" + category + "_" + mode.CustomMode,
                    UseVisualStyleBackColor = true,
                    Enabled = false,
                };
                but.Click += ModeButton_Click;
                section.Buttons.Add(but);
                Controls.Add(but);
            }

            sections.Add(section);
        }

        #region Layout and painting

        private void CreateFonts()
        {
            headerFont?.Dispose();
            statusFont?.Dispose();
            headerFont = new Font(Font, FontStyle.Bold);
            statusFont = new Font(Font.FontFamily, Math.Max(6f, Font.Size - 1f), FontStyle.Italic);
        }

        protected override void OnFontChanged(EventArgs e)
        {
            base.OnFontChanged(e);
            CreateFonts();
            DoLayout();
        }

        protected override void OnVisibleChanged(EventArgs e)
        {
            base.OnVisibleChanged(e);
            if (Visible)
                UpdateState();
        }

        protected override void OnSizeChanged(EventArgs e)
        {
            base.OnSizeChanged(e);
            DoLayout();
        }

        /// <summary>
        /// Height the panel needs to show every section at the given width.
        /// Lets the host decide whether to reserve room for a scroll bar without resizing.
        /// </summary>
        public int GetPreferredHeight(int width)
        {
            return Arrange(width, false);
        }

        /// <summary>
        /// Positions the buttons in a grid that reflows with the available width and
        /// sets this control's height to fit.
        /// </summary>
        private void DoLayout()
        {
            if (layingOut || sections.Count == 0 || headerFont == null)
                return;
            layingOut = true;
            try
            {
                int height = Arrange(ClientSize.Width, true);
                if (Height != height)
                    Height = height;
            }
            finally
            {
                layingOut = false;
            }

            Invalidate();
        }

        private int Arrange(int clientWidth, bool apply)
        {
            if (sections.Count == 0 || headerFont == null)
                return 0;

            int buttonHeight = Font.Height + 10;
            int minButtonWidth = 64;
            foreach (var but in sections.SelectMany(s => s.Buttons))
                minButtonWidth = Math.Max(minButtonWidth,
                    TextRenderer.MeasureText(but.Text, but.Font).Width + 14);

            int left = OuterPad + AccentWidth + 4;
            int usable = Math.Max(minButtonWidth, clientWidth - left - OuterPad);
            int columns = Math.Max(1, (usable + ButtonGap) / (minButtonWidth + ButtonGap));
            int buttonWidth = (usable - (columns - 1) * ButtonGap) / columns;
            int headerHeight = headerFont.Height + 4;

            int y = OuterPad;
            foreach (var section in sections)
            {
                int top = y;
                y += headerHeight;

                if (apply)
                {
                    for (int i = 0; i < section.Buttons.Count; i++)
                    {
                        int col = i % columns;
                        int row = i / columns;
                        section.Buttons[i].SetBounds(left + col * (buttonWidth + ButtonGap),
                            y + row * (buttonHeight + ButtonGap), buttonWidth, buttonHeight);
                    }
                }

                int rows = (section.Buttons.Count + columns - 1) / columns;
                y += rows * (buttonHeight + ButtonGap) - ButtonGap;
                if (apply)
                    section.HeaderBounds = new Rectangle(OuterPad, top, clientWidth - 2 * OuterPad, y - top);
                y += SectionGap;
            }

            return y - SectionGap + OuterPad;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);

            int headerHeight = headerFont.Height + 4;
            foreach (var section in sections)
            {
                var b = section.HeaderBounds;
                var accent = section.Available ? section.Accent : ControlPaint.Light(section.Accent, 0.6f);

                using (var brush = new SolidBrush(accent))
                    e.Graphics.FillRectangle(brush, b.X, b.Y, AccentWidth, b.Height);

                int textX = b.X + AccentWidth + 4;
                TextRenderer.DrawText(e.Graphics, section.Title, headerFont,
                    new Point(textX, b.Y + 1), accent, TextFormatFlags.NoPrefix);

                if (!string.IsNullOrEmpty(section.Status))
                {
                    int titleWidth = TextRenderer.MeasureText(section.Title, headerFont).Width;
                    var statusRect = new Rectangle(textX + titleWidth + 4, b.Y + 1,
                        Math.Max(0, b.Right - (textX + titleWidth + 4)), headerHeight);
                    TextRenderer.DrawText(e.Graphics, section.Status, statusFont, statusRect, ForeColor,
                        TextFormatFlags.NoPrefix | TextFormatFlags.EndEllipsis | TextFormatFlags.Left |
                        TextFormatFlags.VerticalCenter);
                }
            }
        }

        #endregion

        #region Vehicle state

        private static MAVLinkInterface ComPort => MainV2.comPort;

        private static bool IsConnected()
        {
            var port = ComPort;
            return port != null && port.BaseStream != null && port.BaseStream.IsOpen;
        }

        /// <summary>
        /// True when the connected ArduPlane is a QuadPlane: Q_ENABLE >= 1, or the
        /// heartbeat already reports a VTOL MAV_TYPE.
        /// </summary>
        private static bool IsQuadPlane(MAVState mav)
        {
            try
            {
                if (mav.param.ContainsKey("Q_ENABLE") && mav.param["Q_ENABLE"].Value >= 1)
                    return true;
            }
            catch
            {
                // param list being refreshed; fall through to MAV_TYPE check
            }

            int type = (int)mav.aptype;
            return type >= (int)MAVLink.MAV_TYPE.VTOL_DUOROTOR && type <= 25; // VTOL_* MAV_TYPE range
        }

        /// <summary>
        /// Returns whether the category can be commanded right now, and a short reason if not.
        /// </summary>
        private static bool IsCategoryAvailable(ModeCategory category, out string reason)
        {
            if (!IsConnected())
            {
                reason = "not connected";
                return false;
            }

            var mav = ComPort.MAV;
            var firmware = mav.cs.firmware;

            switch (category)
            {
                case ModeCategory.Multicopter:
                    if (firmware == Firmwares.ArduCopter2)
                        break;
                    reason = "needs ArduCopter";
                    return false;

                case ModeCategory.FixedWing:
                    if (firmware == Firmwares.ArduPlane)
                        break;
                    reason = "needs ArduPlane";
                    return false;

                case ModeCategory.Vtol:
                    if (firmware != Firmwares.ArduPlane)
                    {
                        reason = "needs ArduPlane QuadPlane";
                        return false;
                    }
                    if (!IsQuadPlane(mav))
                    {
                        reason = mav.param.Count == 0 ? "waiting for parameters" : "Q_ENABLE is 0";
                        return false;
                    }
                    break;

                case ModeCategory.Rover:
                    if (firmware == Firmwares.ArduRover)
                        break;
                    reason = "needs ArduRover";
                    return false;
            }

            reason = "";
            return true;
        }

        /// <summary>
        /// The vehicle's current custom_mode number, derived from the mode name the
        /// heartbeat handler stores in cs.mode. Cached so the parameter metadata lookup
        /// only runs when the mode or firmware changes.
        /// </summary>
        private int CurrentModeNumber()
        {
            if (!IsConnected())
                return -1;

            var cs = ComPort.MAV.cs;
            if (cs.mode == cachedModeText && cs.firmware == cachedFirmware)
                return cachedModeNumber;

            cachedModeText = cs.mode;
            cachedFirmware = cs.firmware;
            cachedModeNumber = -1;

            try
            {
                var list = Common.getModesList(cs.firmware);
                if (list != null && !string.IsNullOrEmpty(cs.mode))
                {
                    foreach (var pair in list)
                    {
                        if (string.Equals(pair.Value, cs.mode, StringComparison.OrdinalIgnoreCase))
                        {
                            cachedModeNumber = pair.Key;
                            break;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                log.Debug("Mode lookup failed: " + ex.Message);
            }

            return cachedModeNumber;
        }

        private void UpdateState()
        {
            if (IsDisposed || !Visible)
                return;

            int currentMode = CurrentModeNumber();
            bool changed = false;

            foreach (var section in sections)
            {
                string reason;
                bool available = IsCategoryAvailable(section.Category, out reason);

                string status;
                if (!available)
                {
                    status = "(" + reason + ")";
                }
                else if (pendingMode != null && pendingMode.Category == section.Category)
                {
                    if (currentMode == pendingMode.CustomMode)
                    {
                        pendingMode = null;
                        status = "Active: " + ActiveLabel(section, currentMode);
                    }
                    else if ((DateTime.Now - pendingSince).TotalMilliseconds > ConfirmTimeoutMs)
                    {
                        status = pendingMode.Label + " not confirmed by vehicle - check Messages tab";
                    }
                    else
                    {
                        status = "Requesting " + pendingMode.Label + "...";
                    }
                }
                else
                {
                    string label = ActiveLabel(section, currentMode);
                    status = label != null ? "Active: " + label : "Current: " + (ComPort.MAV.cs.mode ?? "?");
                }

                if (section.Available != available || section.Status != status)
                {
                    section.Available = available;
                    section.Status = status;
                    changed = true;
                }

                foreach (var but in section.Buttons)
                {
                    if (but.Enabled != available)
                        but.Enabled = available;

                    var mode = (ModeDefinition)but.Tag;
                    SetActive(but, available && currentMode >= 0 && mode.CustomMode == (uint)currentMode);
                }
            }

            if (changed)
                Invalidate();
        }

        private static string ActiveLabel(Section section, int currentMode)
        {
            if (currentMode < 0)
                return null;
            var but = section.Buttons.FirstOrDefault(b => ((ModeDefinition)b.Tag).CustomMode == (uint)currentMode);
            return but?.Text;
        }

        /// <summary>
        /// Highlights the button for the vehicle's current mode. The themed colours are
        /// remembered so they can be restored, and the highlight is re-applied if a theme
        /// change overwrites it.
        /// </summary>
        private void SetActive(MyButton but, bool active)
        {
            if (active)
            {
                if (!savedColors.ContainsKey(but))
                    savedColors[but] = new[] { but.BGGradTop, but.BGGradBot };
                if (but.BGGradTop != ActiveTop)
                    but.BGGradTop = ActiveTop;
                if (but.BGGradBot != ActiveBottom)
                    but.BGGradBot = ActiveBottom;
            }
            else if (savedColors.TryGetValue(but, out var colors))
            {
                but.BGGradTop = colors[0];
                but.BGGradBot = colors[1];
                savedColors.Remove(but);
            }
        }

        #endregion

        #region Mode request

        private void ModeButton_Click(object sender, EventArgs e)
        {
            var but = sender as MyButton;
            var mode = but?.Tag as ModeDefinition;
            if (mode == null)
                return;

            // Re-check at click time: the vehicle may have changed since the last refresh,
            // and sending one firmware's mode number to another firmware selects a different mode.
            string reason;
            if (!IsCategoryAvailable(mode.Category, out reason))
            {
                CustomMessageBox.Show("Cannot request " + mode.Label + ": " + reason + ".", Strings.ERROR);
                UpdateState();
                return;
            }

            var mav = ComPort.MAV;

            if (mav.cs.failsafe)
            {
                if (CustomMessageBox.Show("You are in failsafe, are you sure?", "Failsafe", MessageBoxButtons.YesNo) !=
                    (int)DialogResult.Yes)
                {
                    return;
                }
            }

            try
            {
                var request = new MAVLink.mavlink_set_mode_t
                {
                    target_system = (byte)mav.sysid,
                    base_mode = (byte)MAVLink.MAV_MODE_FLAG.CUSTOM_MODE_ENABLED,
                    custom_mode = mode.CustomMode
                };

                log.Info("Flight mode button: " + mode.Category + " " + mode.Label + " (custom_mode " +
                         mode.CustomMode + ") sysid " + mav.sysid + " compid " + mav.compid);

                ComPort.setMode(mav.sysid, mav.compid, request);

                pendingMode = mode;
                pendingSince = DateTime.Now;
            }
            catch (Exception ex)
            {
                log.Error("Flight mode request failed", ex);
                CustomMessageBox.Show(Strings.CommandFailed + "\n" + ex.Message, Strings.ERROR);
            }

            UpdateState();
        }

        #endregion

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                stateTimer.Stop();
                stateTimer.Dispose();
                headerFont.Dispose();
                statusFont.Dispose();
            }

            base.Dispose(disposing);
        }
    }
}

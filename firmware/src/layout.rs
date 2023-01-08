use keyberon::action::{k, l, Action, HoldTapAction, HoldTapConfig};
use keyberon::key_code::{KeyCode, KeyCode::*};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Uf2,
    Reset,
}
const BOOTLOAD: Action<CustomActions> = Action::Custom(CustomActions::Uf2);
const RESET: Action<CustomActions> = Action::Custom(CustomActions::Reset);

const A_LS: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(LShift),
    tap: k(A),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const L3_S: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: l(3),
    tap: k(S),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const D_LA: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(LAlt),
    tap: k(D),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const L1_F: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: l(1),
    tap: k(F),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const DT_R: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(RAlt),
    tap: k(Dot),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const X_LA: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(LAlt),
    tap: k(X),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const SL_R: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(RCtrl),
    tap: k(Slash),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const Z_LC: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(LCtrl),
    tap: k(Z),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const L2_C: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: l(2),
    tap: k(C),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const SM_R: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: k(RShift),
    tap: k(SColon),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const L4_S: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: l(4),
    tap: k(Space),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});
const L2_O: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
    timeout: 200,
    hold: l(2),
    tap: k(Comma),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
});

const CT_T: Action<CustomActions> =
    Action::MultipleKeyCodes(&[KeyCode::LCtrl, KeyCode::Tab].as_slice());
const SC_T: Action<CustomActions> =
    Action::MultipleKeyCodes(&[KeyCode::LShift, KeyCode::LCtrl, KeyCode::Tab].as_slice());
const CT_C: Action<CustomActions> =
    Action::MultipleKeyCodes(&[KeyCode::LCtrl, KeyCode::C].as_slice());
const CT_V: Action<CustomActions> =
    Action::MultipleKeyCodes(&[KeyCode::LCtrl, KeyCode::V].as_slice());
const SF_T: Action<CustomActions> =
    Action::MultipleKeyCodes(&[KeyCode::LShift, KeyCode::Tab].as_slice());

// TODO: fix chords to map to tab, escape, and enter
pub const CHORDS: [keyberon::chording::ChordDef; 6] = [
    ((0, 12), &[(0, 8), (1, 8)]),   // Escape
    ((1, 12), &[(0, 9), (1, 9)]),   // Tab
    ((2, 12), &[(0, 10), (1, 10)]), // Enter
    ((0, 12), &[(0, 12), (0, 13)]), // unused
    ((1, 12), &[(1, 12), (1, 13)]), // unused
    ((2, 12), &[(2, 12), (2, 13)]), // unused
];

#[rustfmt::skip]
    pub static LAYERS: keyberon::layout::Layers<40, 1, 5, CustomActions> = keyberon::layout::layout! {
    {[             // 0
        Q          W      E      R      T      Y             U              I            O          P
        {A_LS}     {L3_S} {D_LA} {L1_F} G      H             J              K            L          {SM_R}
        {Z_LC}     {X_LA} {L2_C} V      B      N             M              {L2_O}       {DT_R}     {SL_R}
        t          t      t      t      BSpace {L4_S}        t              t            t          t
    ]}
    {[             // 1
        t          t      t      t      t      *             7              8            9          +
        t          t      t      t      t      /             4              5            6          -
        t          t      t      t      t      .             1              2            3          .
        t          t      t      t      t      0             t              t            t          t
    ]}
    {[             // 2
        !          @      #      $      %      t             ~              |            '`'        +
        '{'        '}'    '('    ')'    t      =             '_'            -            '"'        Quote
        '['        ']'    ^      &      *      t             /              '\\'         t          t
        t          t      t      t      t      t             t              t            t          t
    ]}
    {[             // 3
        t          t      t      t      t      t             {SF_T}         PgUp         Tab        t
        t          t      Delete t      t      Left          Down           Up           Right      Enter
        t          t      t      t      t      t             Home           PgDown       End        t
        t          t      t      t      t      t             t              t            t          t
    ]}
    {[             // 4
        {BOOTLOAD} F7     F8     F9     F10    MediaNextSong MediaPlayPause MediaVolDown MediaVolUp PScreen
        {RESET}    F4     F5     F6     F11    t             Escape         Tab          Enter      Enter
        t          F1     F2     F3     F12    MediaSleep    t              t            t          t
        t          t      t      t      Delete t             t              t            t          t
    ]}
};

use keyberon::action::{
    k, l, m, Action,
    Action::{Custom, HoldTap, Trans},
    HoldTapAction, HoldTapConfig,
};

use keyberon::key_code::KeyCode::*;

const LCTL_ESC: Action<()> = HoldTap(&HoldTapAction {
    timeout: 200,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: k(LCtrl),
    tap: k(Escape),
});

const LCTL_SLASH: Action<()> = HoldTap(&HoldTapAction {
    timeout: 200,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: k(LCtrl),
    tap: k(Slash),
});

const TILD: Action<()> = m(&[LShift, Grave].as_slice());
const EXLM: Action<()> = m(&[LShift, Kb1].as_slice());
const AT: Action<()> = m(&[LShift, Kb2].as_slice());
const HASH: Action<()> = m(&[LShift, Kb3].as_slice());
const DLR: Action<()> = m(&[LShift, Kb4].as_slice());
const PERC: Action<()> = m(&[LShift, Kb5].as_slice());
const CIRC: Action<()> = m(&[LShift, Kb6].as_slice());
const AMPR: Action<()> = m(&[LShift, Kb7].as_slice());
const ASTR: Action<()> = m(&[LShift, Kb8].as_slice());
const LPRN: Action<()> = m(&[LShift, Kb9].as_slice());
const RPRN: Action<()> = m(&[LShift, Kb0].as_slice());
const UNDS: Action<()> = m(&[LShift, Minus].as_slice());
const PLUS: Action<()> = m(&[LShift, Equal].as_slice());
const LCBR: Action<()> = m(&[LShift, LBracket].as_slice());
const RCBR: Action<()> = m(&[LShift, RBracket].as_slice());
const PIPE: Action<()> = m(&[LShift, Bslash].as_slice());
const WSCP_LEFT: Action<()> = m(&[LCtrl, LAlt, Left].as_slice());
const WSCP_RIGHT: Action<()> = m(&[LCtrl, LAlt, Right].as_slice());
const BACK: Action<()> = m(&[LCtrl, LAlt, Minus].as_slice());
const FORWD: Action<()> = m(&[LCtrl, LShift, Minus].as_slice());

#[rustfmt::skip]
pub const LAYERS: keyberon::layout::Layers<14, 4, 4, ()> = [
    [
        [k(Tab),    k(SColon), k(Comma), k(Dot),  k(P), k(Y),     Trans,   Trans,     k(F),      k(G), k(C),    k(R),    k(L),  k(Minus)],
        [LCTL_ESC,  k(A),      k(O),     k(E),    k(U), k(I),     Trans,   Trans,     k(D),      k(H), k(T),    k(N),    k(S),  LCTL_SLASH],
        [k(LShift), k(Quote),  k(Q),     k(J),    k(K), k(X),     l(3),    k(RShift), k(B),      k(M), k(W),    k(V),    k(Z),  k(Enter)],
        [k(Grave),  k(LCtrl),  k(LAlt),  k(LGui), l(1), k(Space), k(RAlt), k(RAlt),   k(BSpace), l(2), k(Left), k(Down), k(Up), k(Right)],
    ],
    [
        [TILD,      EXLM,  AT,    HASH,  DLR,    PERC,   Trans, Trans, CIRC,   AMPR,   ASTR,             LPRN,            RPRN,          k(Delete)],
        [k(Delete), k(F1), k(F2), k(F3), k(F4),  k(F5),  Trans, Trans, k(F6),  UNDS,   PLUS,             LCBR,            RCBR,          PIPE],
        [Trans,     k(F7), k(F8), k(F9), k(F10), k(F11), Trans, Trans, k(F12), k(End), Trans,            Trans,           Trans,         Trans],
        [Trans,     Trans, Trans, Trans, Trans,  Trans,  Trans, Trans, Trans,  Trans,  k(MediaNextSong), k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
    ],
    [
        [k(Grave),  k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), Trans, Trans, k(Kb6), k(Kb7),   k(Kb8),       k(Kb9),          k(Kb0),        k(Delete)],
        [k(Delete), k(F1),  k(F2),  k(F3),  k(F4),  k(F5),  Trans, Trans, k(F6),  k(Minus), k(Equal),     k(LBracket),     k(RBracket),   k(Bslash)],
        [Trans,     k(F7),  k(F8),  k(F9),  k(F10), k(F11), Trans, Trans, k(F12), k(End),   Trans,        Trans,           Trans,         Trans],
        [Trans,     Trans,  Trans,  Trans,  Trans,  Trans,  Trans, Trans, Trans,  Trans,    {Custom(())}, k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
    ],
    [
        [TILD,      EXLM,  AT,    BACK,      FORWD,       PERC,   Trans, Trans, CIRC,      AMPR,    k(Up),            LPRN,            RPRN,          k(Delete)],
        [k(LAlt),   k(F1), k(F2), WSCP_LEFT, WSCP_RIGHT,  k(F5),  Trans, Trans, k(F6),     k(Left), k(Down),          k(Right),        RCBR,          PIPE],
        [Trans,     k(F7), k(F8), k(F9),     k(F10),      k(F11), Trans, Trans, k(F12),    k(Home), k(End),           Trans,           Trans,         Trans],
        [Trans,     Trans, Trans, Trans,     Trans,       Trans,  Trans, Trans, k(PgDown), k(PgUp), k(MediaNextSong), k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
    ],
];

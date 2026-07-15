import math

import numpy as np
import pychrono as chrono


class PayloadDither:
  def __init__(self, my_system, my_frame, contact_material,
               n_micro=16,
               micro_radius=0.008,
               micro_density=2000,
               slot_offsets=None,
               f_fast=8.0,
               f_slow=0.3,
               mean0=8,
               amp=2,
               seed=1):
    self.sys = my_system
    self.frame = my_frame
    self.mat = contact_material
    self.n_micro = int(n_micro)
    self.r = float(micro_radius)
    self.rho = float(micro_density)
    self.f_fast = float(f_fast)
    self.f_slow = float(f_slow)
    self.mean0 = int(mean0)
    self.amp = int(amp)
    self.rng = np.random.default_rng(seed)

    if slot_offsets is None:
      slot_offsets = []
      xs = np.linspace(-0.06, 0.06, 4)
      ys = np.linspace(-0.16, -0.10, 4)
      z = 0.07
      for i in range(self.n_micro):
        slot_offsets.append((float(xs[i % 4]), float(ys[(i // 4) % 4]), float(z)))

    if len(slot_offsets) < self.n_micro:
      raise ValueError("slot_offsets must contain at least n_micro entries.")

    self.slots = slot_offsets[:self.n_micro]
    self.bodies = []
    self.links = [None] * self.n_micro
    self.attached = np.zeros(self.n_micro, dtype=bool)
    self.log = []

  def _mk_micro_body(self, i):
    body = chrono.ChBodyEasySphere(self.r, self.rho, True, True, self.mat)
    body.SetName(f"MicroPayload_{i + 1}")
    self.sys.Add(body)
    return body

  def _frame_to_world(self, off_xyz):
    off = chrono.ChVector3d(off_xyz[0], off_xyz[1], off_xyz[2])
    if hasattr(self.frame, "Point_Body2World"):
      return self.frame.Point_Body2World(off)
    return self.frame.GetFrameRefToAbs().TransformPointLocalToParent(off)

  @staticmethod
  def _call_first(obj, names):
    for name in names:
      if hasattr(obj, name):
        return getattr(obj, name)()
    return None

  @staticmethod
  def _set_first(obj, names, value):
    if value is None:
      return
    for name in names:
      if hasattr(obj, name):
        getattr(obj, name)(value)
        return

  def _attach_i(self, i):
    if self.attached[i]:
      return

    body = self.bodies[i]
    body.SetPos(self._frame_to_world(self.slots[i]))
    body.SetRot(self.frame.GetRot())

    link = chrono.ChLinkMateFix()
    link.Initialize(body, self.frame)
    self.sys.Add(link)

    self.links[i] = link
    self.attached[i] = True

  def _detach_i(self, i):
    if not self.attached[i]:
      return

    link = self.links[i]
    if link is not None:
      self.sys.Remove(link)
    self.links[i] = None
    self.attached[i] = False

    body = self.bodies[i]
    self._set_first(
      body,
      ("SetPosDt", "SetPos_dt"),
      self._call_first(self.frame, ("GetPosDt", "GetPos_dt"))
    )
    self._set_first(
      body,
      ("SetAngVelParent", "SetWvel_par"),
      self._call_first(self.frame, ("GetAngVelParent", "GetWvel_par"))
    )

  def initialize(self, start_attached=None):
    self.bodies = [self._mk_micro_body(i) for i in range(self.n_micro)]
    if start_attached is None:
      start_attached = self.mean0
    start_attached = max(0, min(self.n_micro, int(start_attached)))

    for i in range(start_attached):
      self._attach_i(i)

  def desired_k(self, t):
    k_mean = self.mean0 + int(round(self.amp * math.sin(2 * math.pi * self.f_slow * t)))
    k_mean = max(0, min(self.n_micro, k_mean))

    fast = math.sin(2 * math.pi * self.f_fast * t)
    k = k_mean + (1 if fast >= 0 else 0)
    k = max(0, min(self.n_micro, k))
    return k_mean, k

  def update(self, t):
    k_mean, k_target = self.desired_k(t)
    k_now = int(self.attached.sum())

    if k_now < k_target:
      for i in range(self.n_micro):
        if not self.attached[i]:
          self._attach_i(i)
          k_now += 1
          if k_now == k_target:
            break
    elif k_now > k_target:
      for i in range(self.n_micro - 1, -1, -1):
        if self.attached[i]:
          self._detach_i(i)
          k_now -= 1
          if k_now == k_target:
            break

    self.log.append([t, k_mean, int(self.attached.sum())])

  def to_log_dict(self):
    data = np.array(self.log, dtype=float) if self.log else np.zeros((0, 3))
    return {
      "time": data[:, 0].reshape(-1, 1),
      "k_mean": data[:, 1].reshape(-1, 1),
      "k_attached": data[:, 2].reshape(-1, 1),
    }


class ThrustDither:
  def __init__(self,
               motor_ids=(0,),
               f_fast=10.0,
               f_slow=0.2,
               eps_fast=0.03,
               eps_slow=0.02,
               alpha0=1.00,
               enforce_Tmin_Tmax=True):
    self.motor_ids = list(motor_ids)
    self.f_fast = float(f_fast)
    self.f_slow = float(f_slow)
    self.eps_fast = float(eps_fast)
    self.eps_slow = float(eps_slow)
    self.alpha0 = float(alpha0)
    self.enforce = bool(enforce_Tmin_Tmax)
    self.log = []

  def alpha(self, t):
    slow = math.sin(2 * math.pi * self.f_slow * t)
    fast_sign = 1.0 if math.sin(2 * math.pi * self.f_fast * t) >= 0 else -1.0
    alpha = self.alpha0 * (1.0 + self.eps_slow * slow + self.eps_fast * fast_sign)
    return max(0.0, alpha), fast_sign

  def apply(self, T, t, T_MIN, T_MAX):
    T_is_flat = (T.ndim == 1)
    if T_is_flat:
      T = T.reshape((-1, 1))

    alpha, sign = self.alpha(t)
    for i in self.motor_ids:
      if 0 <= i < T.shape[0]:
        T[i, 0] *= alpha
        if self.enforce:
          T[i, 0] = max(T_MIN, min(T[i, 0], T_MAX))

    self.log.append([t, alpha, sign])
    return T.reshape((-1,)) if T_is_flat else T

  def to_log_dict(self):
    data = np.array(self.log, dtype=float) if self.log else np.zeros((0, 3))
    return {
      "time": data[:, 0].reshape(-1, 1),
      "alpha": data[:, 1].reshape(-1, 1),
      "active_sign": data[:, 2].reshape(-1, 1),
    }

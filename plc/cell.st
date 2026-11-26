PROGRAM cell_sequencer
  VAR
    initialized : BOOL := FALSE;
    pidx : INT := 0;
    started : BOOL := FALSE;
    start_ms : DINT := 0;
    clock_ms : DINT := 0;
    elapsed_ms : DINT := 0;
    context : INT := 0;
    gripper : INT := 0;
    seq : UINT := 0;
    completed : INT := 0;
    done : BOOL := FALSE;
    first : BOOL := FALSE;
    j : INT := 0;
    hold : ARRAY[0..6] OF UINT;
    qbuf : ARRAY[0..6] OF UINT;
    teach : ARRAY[0..5, 0..6] OF UINT;
    prog_kind : ARRAY[0..25] OF INT;
    prog_teach : ARRAY[0..25] OF INT;
    prog_ms : ARRAY[0..25] OF DINT;
    prog_ctx : ARRAY[0..25] OF INT;
    prog_grip : ARRAY[0..25] OF INT;
  END_VAR
  VAR
    out_seq AT %QW0 : UINT;
    out_step AT %QW1 : UINT;
    out_kind AT %QW2 : UINT;
    out_ms AT %QW3 : UINT;
    out_ctx AT %QW4 : UINT;
    out_grip AT %QW5 : UINT;
    out_passes AT %QW6 : UINT;
    out_finished AT %QW7 : UINT;
    out_q0 AT %QW10 : UINT;
    out_q1 AT %QW11 : UINT;
    out_q2 AT %QW12 : UINT;
    out_q3 AT %QW13 : UINT;
    out_q4 AT %QW14 : UINT;
    out_q5 AT %QW15 : UINT;
    out_q6 AT %QW16 : UINT;
    run_coil AT %QX0.0 : BOOL;
    alarm_coil AT %QX1.0 : BOOL;
    safe_coil AT %QX1.1 : BOOL;
  END_VAR

  IF NOT initialized THEN
      teach[0,0] := 33095;
      teach[0,1] := 41604;
      teach[0,2] := 30153;
      teach[0,3] := 24589;
      teach[0,4] := 31495;
      teach[0,5] := 40387;
      teach[0,6] := 35622;
      teach[1,0] := 32913;
      teach[1,1] := 41703;
      teach[1,2] := 29208;
      teach[1,3] := 25096;
      teach[1,4] := 31571;
      teach[1,5] := 40046;
      teach[1,6] := 36198;
      teach[2,0] := 31093;
      teach[2,1] := 36388;
      teach[2,2] := 35514;
      teach[2,3] := 17735;
      teach[2,4] := 33801;
      teach[2,5] := 42828;
      teach[2,6] := 30503;
      teach[3,0] := 30938;
      teach[3,1] := 36654;
      teach[3,2] := 36449;
      teach[3,3] := 18101;
      teach[3,4] := 33584;
      teach[3,5] := 42203;
      teach[3,6] := 30195;
      teach[4,0] := 30203;
      teach[4,1] := 35938;
      teach[4,2] := 35829;
      teach[4,3] := 17860;
      teach[4,4] := 33101;
      teach[4,5] := 42694;
      teach[4,6] := 30842;
      teach[5,0] := 32701;
      teach[5,1] := 41511;
      teach[5,2] := 29421;
      teach[5,3] := 24395;
      teach[5,4] := 30902;
      teach[5,5] := 39911;
      teach[5,6] := 36122;
      hold[0] := 33000;
      hold[1] := 38000;
      hold[2] := 33000;
      hold[3] := 21000;
      hold[4] := 33000;
      hold[5] := 41000;
      hold[6] := 33000;
      prog_kind[0] := 1; prog_teach[0] := 0; prog_ms[0] := 4000; prog_ctx[0] := -1; prog_grip[0] := 0;
      prog_kind[1] := 1; prog_teach[1] := 1; prog_ms[1] := 2000; prog_ctx[1] := -1; prog_grip[1] := 0;
      prog_kind[2] := 2; prog_teach[2] := -1; prog_ms[2] := 500; prog_ctx[2] := -1; prog_grip[2] := 0;
      prog_kind[3] := 0; prog_teach[3] := -1; prog_ms[3] := 0; prog_ctx[3] := 1; prog_grip[3] := 0;
      prog_kind[4] := 0; prog_teach[4] := -1; prog_ms[4] := 0; prog_ctx[4] := -1; prog_grip[4] := 1;
      prog_kind[5] := 1; prog_teach[5] := 0; prog_ms[5] := 2000; prog_ctx[5] := -1; prog_grip[5] := 0;
      prog_kind[6] := 1; prog_teach[6] := 2; prog_ms[6] := 4000; prog_ctx[6] := -1; prog_grip[6] := 0;
      prog_kind[7] := 1; prog_teach[7] := 3; prog_ms[7] := 2000; prog_ctx[7] := -1; prog_grip[7] := 0;
      prog_kind[8] := 0; prog_teach[8] := -1; prog_ms[8] := 0; prog_ctx[8] := 0; prog_grip[8] := 0;
      prog_kind[9] := 0; prog_teach[9] := -1; prog_ms[9] := 0; prog_ctx[9] := -1; prog_grip[9] := 2;
      prog_kind[10] := 2; prog_teach[10] := -1; prog_ms[10] := 500; prog_ctx[10] := -1; prog_grip[10] := 0;
      prog_kind[11] := 0; prog_teach[11] := -1; prog_ms[11] := 0; prog_ctx[11] := -1; prog_grip[11] := 3;
      prog_kind[12] := 1; prog_teach[12] := 2; prog_ms[12] := 2000; prog_ctx[12] := -1; prog_grip[12] := 0;
      prog_kind[13] := 1; prog_teach[13] := 2; prog_ms[13] := 4000; prog_ctx[13] := -1; prog_grip[13] := 0;
      prog_kind[14] := 1; prog_teach[14] := 4; prog_ms[14] := 2000; prog_ctx[14] := -1; prog_grip[14] := 0;
      prog_kind[15] := 2; prog_teach[15] := -1; prog_ms[15] := 500; prog_ctx[15] := -1; prog_grip[15] := 0;
      prog_kind[16] := 0; prog_teach[16] := -1; prog_ms[16] := 0; prog_ctx[16] := 1; prog_grip[16] := 0;
      prog_kind[17] := 0; prog_teach[17] := -1; prog_ms[17] := 0; prog_ctx[17] := -1; prog_grip[17] := 1;
      prog_kind[18] := 1; prog_teach[18] := 2; prog_ms[18] := 2000; prog_ctx[18] := -1; prog_grip[18] := 0;
      prog_kind[19] := 1; prog_teach[19] := 0; prog_ms[19] := 4000; prog_ctx[19] := -1; prog_grip[19] := 0;
      prog_kind[20] := 1; prog_teach[20] := 5; prog_ms[20] := 2000; prog_ctx[20] := -1; prog_grip[20] := 0;
      prog_kind[21] := 0; prog_teach[21] := -1; prog_ms[21] := 0; prog_ctx[21] := 0; prog_grip[21] := 0;
      prog_kind[22] := 0; prog_teach[22] := -1; prog_ms[22] := 0; prog_ctx[22] := -1; prog_grip[22] := 2;
      prog_kind[23] := 2; prog_teach[23] := -1; prog_ms[23] := 500; prog_ctx[23] := -1; prog_grip[23] := 0;
      prog_kind[24] := 0; prog_teach[24] := -1; prog_ms[24] := 0; prog_ctx[24] := -1; prog_grip[24] := 3;
      prog_kind[25] := 1; prog_teach[25] := 0; prog_ms[25] := 2000; prog_ctx[25] := -1; prog_grip[25] := 0;
      FOR j := 0 TO 6 DO qbuf[j] := hold[j]; END_FOR;
      run_coil := TRUE;
      initialized := TRUE;
  END_IF;

  clock_ms := clock_ms + 10;
  seq := seq + 1;

  IF alarm_coil THEN
    safe_coil := TRUE;
  ELSE
    safe_coil := FALSE;
    IF NOT started THEN
      started := TRUE;
      start_ms := clock_ms;
      first := TRUE;
    ELSE
      first := FALSE;
    END_IF;
    IF first THEN
      IF prog_grip[pidx] = 1 THEN gripper := 1;
      ELSIF prog_grip[pidx] = 2 THEN gripper := 0;
      END_IF;
      IF prog_ctx[pidx] >= 0 THEN context := prog_ctx[pidx]; END_IF;
    END_IF;
    elapsed_ms := clock_ms - start_ms;
    done := TRUE;
    out_kind := 0;
    out_ms := 0;
    IF prog_kind[pidx] = 1 THEN
      out_kind := 1;
      out_ms := DINT_TO_UINT(prog_ms[pidx]);
      FOR j := 0 TO 6 DO qbuf[j] := teach[prog_teach[pidx], j]; END_FOR;
      IF elapsed_ms < prog_ms[pidx] THEN
        done := FALSE;
      ELSE
        FOR j := 0 TO 6 DO hold[j] := teach[prog_teach[pidx], j]; END_FOR;
      END_IF;
    ELSIF prog_kind[pidx] = 2 THEN
      out_kind := 2;
      FOR j := 0 TO 6 DO qbuf[j] := hold[j]; END_FOR;
      IF elapsed_ms < prog_ms[pidx] THEN done := FALSE; END_IF;
    ELSE
      FOR j := 0 TO 6 DO qbuf[j] := hold[j]; END_FOR;
    END_IF;
    out_step := INT_TO_UINT(pidx);
    IF done THEN
      started := FALSE;
      pidx := pidx + 1;
      IF pidx >= 26 THEN
        pidx := 0;
        completed := completed + 1;
      END_IF;
    END_IF;
  END_IF;

  out_seq := seq;
  out_ctx := INT_TO_UINT(context);
  out_grip := INT_TO_UINT(gripper);
  out_passes := INT_TO_UINT(completed);
  out_q0 := qbuf[0];
  out_q1 := qbuf[1];
  out_q2 := qbuf[2];
  out_q3 := qbuf[3];
  out_q4 := qbuf[4];
  out_q5 := qbuf[5];
  out_q6 := qbuf[6];
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK Main(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM Inst0 WITH Main : cell_sequencer;
  END_RESOURCE
END_CONFIGURATION
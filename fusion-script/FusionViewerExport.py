# FusionViewerExport
# Esporta il design attivo (mesh dei componenti, posizioni e joint)
# in un file .fview.json da aprire con il Visualizzatore Fusion sul tablet.
#
# Unita' nel file: centimetri (unita' interne di Fusion), angoli in radianti.

import adsk.core
import adsk.fusion
import traceback
import json
import base64
import struct

FORMAT_VERSION = 1


# ---------------------------------------------------------------- utilita'

def b64_floats(values):
    return base64.b64encode(struct.pack('<%df' % len(values), *values)).decode('ascii')


def b64_uints(values):
    return base64.b64encode(struct.pack('<%dI' % len(values), *values)).decode('ascii')


def matrix_rows(m):
    """Matrice 4x4 come 16 valori in ordine per righe (traslazione in colonna 4)."""
    out = []
    for r in range(4):
        for c in range(4):
            out.append(m.getCell(r, c))
    return out


def pt(p, m=None):
    if p is None:
        return None
    q = p.copy()
    if m is not None:
        q.transformBy(m)
    return [q.x, q.y, q.z]


def vec(v, m=None):
    if v is None:
        return None
    q = v.copy()
    if m is not None:
        q.transformBy(m)
    q.normalize()
    return [q.x, q.y, q.z]


def color_of(appearance):
    if appearance is None:
        return None
    try:
        props = appearance.appearanceProperties
        # prima prova le proprieta' colore piu' comuni
        for pid in ('opaque_albedo', 'transparent_color', 'metal_f0', 'layered_diffuse', 'generic_diffuse'):
            try:
                p = props.itemById(pid)
                cp = adsk.core.ColorProperty.cast(p)
                if cp and cp.value:
                    c = cp.value
                    return [c.red / 255.0, c.green / 255.0, c.blue / 255.0, c.opacity / 255.0]
            except Exception:
                pass
        for i in range(props.count):
            cp = adsk.core.ColorProperty.cast(props.item(i))
            if cp and cp.value:
                c = cp.value
                return [c.red / 255.0, c.green / 255.0, c.blue / 255.0, c.opacity / 255.0]
    except Exception:
        pass
    return None


def limits(l):
    if l is None:
        return None, None
    mn = mx = None
    try:
        if l.isMinimumValueEnabled:
            mn = l.minimumValue
        if l.isMaximumValueEnabled:
            mx = l.maximumValue
    except Exception:
        pass
    return mn, mx


def joint_frame(g, m):
    """Ritorna origine e assi (x, y, z) di una JointGeometry o JointOrigin, in coordinate mondo."""
    if g is None:
        return None
    jo = adsk.fusion.JointOrigin.cast(g)
    if jo:
        g = jo.geometry
    jg = adsk.fusion.JointGeometry.cast(g)
    if not jg:
        return None
    try:
        return {
            'origin': pt(jg.origin, m),
            'z': vec(jg.primaryAxisVector, m),
            'x': vec(jg.secondaryAxisVector, m),
            'y': vec(jg.thirdAxisVector, m),
        }
    except Exception:
        return None


def direction(enum_value, custom_vector, frame, m):
    D = adsk.fusion.JointDirections
    try:
        if enum_value == D.XAxisJointDirection:
            return frame['x']
        if enum_value == D.YAxisJointDirection:
            return frame['y']
        if enum_value == D.ZAxisJointDirection:
            return frame['z']
    except Exception:
        pass
    if custom_vector is not None:
        try:
            return vec(custom_vector, m)
        except Exception:
            pass
    return frame['z']


def g(obj, name, default=None):
    try:
        return getattr(obj, name)
    except Exception:
        return default


def dofs_for(motion, frame, m):
    """Gradi di liberta' del joint: lista di {kind, axis, pivot, value, min, max, label}."""
    T = adsk.fusion.JointTypes
    jt = motion.jointType
    o = frame['origin']
    out = []

    def rot(label, axis, value, lim):
        mn, mx = limits(lim)
        out.append({'kind': 'rot', 'label': label, 'axis': axis, 'pivot': o,
                    'value': value or 0.0, 'min': mn, 'max': mx})

    def slide(label, axis, value, lim):
        mn, mx = limits(lim)
        out.append({'kind': 'slide', 'label': label, 'axis': axis, 'pivot': o,
                    'value': value or 0.0, 'min': mn, 'max': mx})

    if jt == T.RigidJointType:
        return 'rigid', out
    if jt == T.RevoluteJointType:
        mo = adsk.fusion.RevoluteJointMotion.cast(motion)
        rot('Rotazione', direction(g(mo, 'rotationAxis'), g(mo, 'rotationAxisVector'), frame, m),
            g(mo, 'rotationValue'), g(mo, 'rotationLimits'))
        return 'revolute', out
    if jt == T.SliderJointType:
        mo = adsk.fusion.SliderJointMotion.cast(motion)
        slide('Scorrimento', direction(g(mo, 'slideDirection'), g(mo, 'slideDirectionVector'), frame, m),
              g(mo, 'slideValue'), g(mo, 'slideLimits'))
        return 'slider', out
    if jt == T.CylindricalJointType:
        mo = adsk.fusion.CylindricalJointMotion.cast(motion)
        ax = direction(g(mo, 'rotationAxis'), g(mo, 'rotationAxisVector'), frame, m)
        slide('Scorrimento', ax, g(mo, 'slideValue'), g(mo, 'slideLimits'))
        rot('Rotazione', ax, g(mo, 'rotationValue'), g(mo, 'rotationLimits'))
        return 'cylindrical', out
    if jt == T.PinSlotJointType:
        mo = adsk.fusion.PinSlotJointMotion.cast(motion)
        slide('Scorrimento', direction(g(mo, 'slideDirection'), g(mo, 'slideDirectionVector'), frame, m),
              g(mo, 'slideValue'), g(mo, 'slideLimits'))
        rot('Rotazione', direction(g(mo, 'rotationAxis'), g(mo, 'rotationAxisVector'), frame, m),
            g(mo, 'rotationValue'), g(mo, 'rotationLimits'))
        return 'pinslot', out
    if jt == T.PlanarJointType:
        mo = adsk.fusion.PlanarJointMotion.cast(motion)
        n = direction(g(mo, 'normalDirection'), g(mo, 'normalDirectionVector'), frame, m)
        p1 = g(mo, 'primarySlideDirectionVector')
        p1 = vec(p1, m) if p1 is not None else frame['x']
        p2 = g(mo, 'secondarySlideDirectionVector')
        p2 = vec(p2, m) if p2 is not None else frame['y']
        slide('Scorrimento 1', p1, g(mo, 'primarySlideValue'), g(mo, 'primarySlideLimits'))
        slide('Scorrimento 2', p2, g(mo, 'secondarySlideValue'), g(mo, 'secondarySlideLimits'))
        rot('Rotazione', n, g(mo, 'rotationValue'), g(mo, 'rotationLimits'))
        return 'planar', out
    if jt == T.BallJointType:
        mo = adsk.fusion.BallJointMotion.cast(motion)

        def bv(name, fallback):
            v = g(mo, name)
            return vec(v, m) if v is not None else frame[fallback]
        rot('Imbardata', bv('yawDirectionVector', 'z'), g(mo, 'yawValue'), g(mo, 'yawLimits'))
        rot('Beccheggio', bv('pitchDirectionVector', 'x'), g(mo, 'pitchValue'), g(mo, 'pitchLimits'))
        rot('Rollio', bv('rollDirectionVector', 'y'), g(mo, 'rollValue'), g(mo, 'rollLimits'))
        return 'ball', out
    return 'unknown', out


def occ_path(o):
    try:
        return o.fullPathName if o else None
    except Exception:
        return None


# ---------------------------------------------------------------- esportazione

def export_design(design, quality, progress):
    root = design.rootComponent
    occs = [o for o in root.allOccurrences]

    meshes = []          # geometrie (una per corpo di componente, riusate dalle istanze)
    mesh_index = {}
    parts = []
    joints = []
    rigid_groups = []
    warnings = []

    # conteggio per la barra di avanzamento
    total = root.bRepBodies.count + sum(o.component.bRepBodies.count for o in occs)
    progress.maximumValue = max(total, 1)
    done = [0]

    def mesh_for(comp, idx, body):
        key = '%s:%d' % (g(comp, 'id', comp.name), idx)
        if key in mesh_index:
            return mesh_index[key]
        calc = body.meshManager.createMeshCalculator()
        calc.setQuality(quality)
        tm = calc.calculate()
        if tm is None or tm.nodeCount == 0:
            mesh_index[key] = None
            return None
        mid = 'm%d' % len(meshes)
        meshes.append({
            'id': mid,
            'pos': b64_floats(tm.nodeCoordinatesAsFloat),
            'nrm': b64_floats(tm.normalVectorsAsFloat),
            'idx': b64_uints(tm.nodeIndices),
        })
        mesh_index[key] = mid
        return mid

    def bodies_of(comp, proxy_bodies, occ):
        res = []
        for i in range(comp.bRepBodies.count):
            if progress.wasCancelled:
                raise KeyboardInterrupt()
            body = comp.bRepBodies.item(i)
            visible = True
            try:
                pb = proxy_bodies.item(i) if proxy_bodies is not None else body
                visible = pb.isVisible
            except Exception:
                pass
            done[0] += 1
            progress.progressValue = done[0]
            adsk.doEvents()
            if not visible:
                continue
            try:
                mid = mesh_for(comp, i, body)
            except Exception:
                warnings.append('Mesh non calcolata: %s / %s' % (comp.name, body.name))
                continue
            if mid is None:
                continue
            col = None
            try:
                if occ is not None and occ.appearance is not None:
                    col = color_of(occ.appearance)
            except Exception:
                pass
            if col is None:
                col = color_of(g(body, 'appearance'))
            res.append({'mesh': mid, 'name': body.name, 'color': col})
        return res

    # componente radice
    parts.append({
        'id': 'root', 'name': root.name, 'parent': None, 'grounded': True,
        'matrix': [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        'bodies': bodies_of(root, None, None),
    })

    for o in occs:
        path = o.fullPathName
        parent = path.rsplit('+', 1)[0] if '+' in path else 'root'
        try:
            visible = o.isVisible
        except Exception:
            visible = True
        parts.append({
            'id': path,
            'name': o.name,
            'component': o.component.name,
            'parent': parent,
            'grounded': bool(g(o, 'isGrounded', False)),
            'visible': visible,
            'matrix': matrix_rows(o.transform2),
            'bodies': bodies_of(o.component, o.bRepBodies, o),
        })

    # joint: quelli della radice + quelli dentro ogni sotto-assieme
    contexts = [(root, None)] + [(o.component, o) for o in occs]
    seen_components = {}
    for comp, ctx in contexts:
        m = ctx.transform2 if ctx is not None else None
        ctx_path = ctx.fullPathName if ctx is not None else 'root'

        def resolve(native_joint, which):
            # percorso completo dell'occorrenza 'one' o 'two' nel contesto della radice
            proxy = None
            if ctx is not None:
                try:
                    proxy = native_joint.createForAssemblyContext(ctx)
                except Exception:
                    proxy = None
            src = proxy if proxy is not None else native_joint
            occ = g(src, 'occurrenceOne' if which == 1 else 'occurrenceTwo')
            if occ is None:
                return ctx_path
            p = occ_path(occ)
            if proxy is None and ctx is not None and p and not p.startswith(ctx_path + '+'):
                p = ctx_path + '+' + p
            return p

        for coll_name in ('joints', 'asBuiltJoints'):
            coll = g(comp, coll_name)
            if coll is None:
                continue
            for i in range(coll.count):
                j = coll.item(i)
                try:
                    if g(j, 'isSuppressed', False):
                        continue
                    motion = j.jointMotion
                    if coll_name == 'joints':
                        frame = joint_frame(g(j, 'geometryOrOriginTwo'), m) or joint_frame(g(j, 'geometryOrOriginOne'), m)
                    else:
                        frame = joint_frame(g(j, 'geometry'), m)
                    if frame is None:
                        # as-built rigido o geometria non leggibile
                        frame = {'origin': [0, 0, 0], 'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
                    jtype, dofs = dofs_for(motion, frame, m)
                    joints.append({
                        'name': j.name,
                        'type': jtype,
                        'asBuilt': coll_name == 'asBuiltJoints',
                        'one': resolve(j, 1),
                        'two': resolve(j, 2),
                        'locked': bool(g(j, 'isLocked', False)),
                        'frame': frame,
                        'dofs': dofs,
                    })
                except Exception:
                    warnings.append('Joint non letto: %s' % g(j, 'name', '?'))

        rgs = g(comp, 'rigidGroups')
        if rgs is not None:
            for i in range(rgs.count):
                rg = rgs.item(i)
                try:
                    if g(rg, 'isSuppressed', False):
                        continue
                    members = []
                    for k in range(rg.occurrences.count):
                        oc = rg.occurrences.item(k)
                        p = occ_path(oc)
                        if ctx is not None and p and not p.startswith(ctx_path + '+'):
                            p = ctx_path + '+' + p
                        members.append(p)
                    rigid_groups.append(members)
                except Exception:
                    warnings.append('Gruppo rigido non letto')

    try:
        default_units = design.unitsManager.defaultLengthUnits
    except Exception:
        default_units = 'mm'

    # orientamento di modellazione (Y o Z in alto) dalle preferenze di Fusion
    up = 'z'
    try:
        o = adsk.core.Application.get().preferences.generalPreferences.defaultModelingOrientation
        up = 'y' if o == adsk.core.DefaultModelingOrientations.YUpModelingOrientation else 'z'
    except Exception:
        pass

    return {
        'format': 'fview',
        'version': FORMAT_VERSION,
        'name': design.parentDocument.name if design.parentDocument else root.name,
        'units': 'cm',
        'displayUnits': default_units,
        'up': up,
        'meshes': meshes,
        'parts': parts,
        'joints': joints,
        'rigidGroups': rigid_groups,
        'warnings': warnings,
    }


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    progress = None
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Apri un design (area Progettazione) prima di lanciare lo script.', 'Esporta per Visualizzatore')
            return

        res = ui.inputBox('Qualita\' delle mesh:\n1 = bassa (file piccolo)\n2 = normale\n3 = alta\n4 = molto alta',
                          'Esporta per Visualizzatore', '2')
        if isinstance(res, (list, tuple)):
            value, cancelled = res[0], res[1]
        else:
            value, cancelled = res, False
        if cancelled:
            return
        Q = adsk.fusion.TriangleMeshQualityOptions
        quality = {
            '1': Q.LowQualityTriangleMesh,
            '2': Q.NormalQualityTriangleMesh,
            '3': Q.HighQualityTriangleMesh,
            '4': Q.VeryHighQualityTriangleMesh,
        }.get(str(value).strip(), Q.NormalQualityTriangleMesh)

        dlg = ui.createFileDialog()
        dlg.title = 'Salva per il Visualizzatore'
        dlg.filter = 'Visualizzatore Fusion (*.fview.json)'
        name = design.parentDocument.name if design.parentDocument else 'modello'
        dlg.initialFilename = ''.join(ch if ch not in '\\/:*?"<>|' else '_' for ch in name) + '.fview.json'
        if dlg.showSave() != adsk.core.DialogResults.DialogOK:
            return
        path = dlg.filename
        if not path.lower().endswith('.json'):
            path += '.fview.json'

        progress = ui.createProgressDialog()
        progress.cancelButtonText = 'Annulla'
        progress.isBackgroundTranslucent = False
        progress.show('Esporta per Visualizzatore', 'Calcolo mesh: %v di %m corpi', 0, 1, 1)

        try:
            data = export_design(design, quality, progress)
        except KeyboardInterrupt:
            progress.hide()
            ui.messageBox('Esportazione annullata.')
            return

        progress.message = 'Scrittura file...'
        adsk.doEvents()
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, separators=(',', ':'))
        progress.hide()
        progress = None

        movable = [j for j in data['joints'] if j['type'] not in ('rigid', 'unknown') and not j['locked']]
        msg = 'Fatto!\n\n%d parti, %d mesh, %d joint (%d mobili).\n\nFile: %s\n\nCopialo sul tablet e aprilo dal Visualizzatore.' % (
            len(data['parts']), len(data['meshes']), len(data['joints']), len(movable), path)
        if data['warnings']:
            msg += '\n\nAvvisi:\n- ' + '\n- '.join(data['warnings'][:10])
        ui.messageBox(msg, 'Esporta per Visualizzatore')
    except Exception:
        if progress:
            try:
                progress.hide()
            except Exception:
                pass
        if ui:
            ui.messageBox('Errore:\n{}'.format(traceback.format_exc()))
